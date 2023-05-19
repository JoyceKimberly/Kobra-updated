# Patches to the DDL

to make the DDL work better with platformio, some patches are applied.
These range from minor changes to adapt the ddl to the build system, to changes to make the ddl compile with a modern gcc version without warnings.

## `/ld/hc32f46x_param.ld`

the linker script needs minor adjustments to work with the platformio build system, mainly to add support for setting the flash start address and size from the platformio build system.

### 1. Parameters Documentation

while not strictly necessary, it is nice to have the parameters documented in the linker script.
The following comment is added to the top of the linker script:

```cpp
/*
 * parameterized linker flag for the HC32F46x SOC.
 * flash size and start offset have to be set as symbols for the linker script, using
 * the '--defsym' ('-Wl,'--defsym=<SYMBOL_NAME>=<SYMBOL_VALUE>'') flag.
 *
 * note: previously, ram options were also configurable.
 * this was removed to simplify the linker script (all HC32F460 have the same ram anyway)
 */

/* FLASH_START = 0x00000000; */
/* FLASH_SIZE = 256K; */
```

### 2. Flash Start Address and Size

to set the flash start address, the memory region `FLASH` is updated to use the `FLASH_START` symbol as ORIGIN and the `FLASH_SIZE` symbol as LENGTH:

```cpp
MEMORY
{
    FLASH       (rx): ORIGIN = FLASH_START, LENGTH = FLASH_SIZE
    OTP         (rx): ORIGIN = 0x03000C00, LENGTH = 1020
    RAM        (rwx): ORIGIN = 0x1FFF8000, LENGTH = 188K
    RET_RAM    (rwx): ORIGIN = 0x200F0000, LENGTH = 4K
}
```

all other memory regions are left untouched.

### 3. `.vector` and `.icg_sec` fixed size

the `.vector` and `.icg_sec` sections are defined with a fixed size of 1024 and 32 bytes respectively.
This is done by manually setting the program counter to the desired address after the section contents:

```cpp
SECTIONS
{
    /* 1024 bytes fixed size */
    .vectors :
    {
        KEEP(*(.vectors))
        . = 1024;
    } >FLASH


    /* 32 bytes fixed size */
    .icg_sec :
    {
        KEEP(*(.icg_sec))
        . = 1024 + 32;
    } >FLASH

    /* append all other FLASH sections after */
    .text :
    // ...
}
```

## `/cores/ddl/startup/startup_hc32f460.S`

the startup code needs adjustments to work correctly with libc.

### 1. `__libc_init_array` call

to initialize libc, a call to `__libc_init_array` is added before calling `main`:

```assembly
    /* Call the clock system initialization function. */
    bl          SystemInit
    /* call __libc_init_array before calling main */
    bl          __libc_init_array
    /* Call the application's entry point. */
    bl          main
```

## `/cores/ddl/library/src/hc32f460_timer0.c`

the timer0 driver creates a warning when compiled with a modern gcc version.
While it does work, this quiets the build process making real errors easier to spot.

### 1. Patch `TIMER0_BaseInit`

during reset of BCONR register, the original ddl code uses a bitfield `pstcTim0Reg->BCONR_f` cast to a uint32_t to clear the register. This causes a type-pruning warning.
To fix this, `pstcTim0Reg->BCONR_f` can be replaced with `pstcTim0Reg->BCONR` (which is the raw register value BCONR_f represents) in the register clear code.

Since the following code uses the value in the variable `stcBconrTmp`, it has to be set after the register clear code.

See the following diff for the changes:

```diff
@@ -648,6 +648,7 @@ en_result_t TIMER0_BaseInit(M4_TMR0_TypeDef* pstcTim0Reg,en_tim0_channel_t enCh,
                        const stc_tim0_base_init_t* pstcBaseInit)
 {
     stc_tmr0_bconr_field_t stcBconrTmp;
+    uint32_t stcBconrTmpRaw;
     en_result_t enRet = Ok;
     uint32_t u32TimeOut = 0ul;

@@ -666,19 +667,19 @@ en_result_t TIMER0_BaseInit(M4_TMR0_TypeDef* pstcTim0Reg,en_tim0_channel_t enCh,
         }

         /*Read current BCONR register */
-        stcBconrTmp = pstcTim0Reg->BCONR_f;
+        stcBconrTmpRaw = pstcTim0Reg->BCONR;
         /* Clear current configurate CH */
         if(Tim0_ChannelA == enCh)
         {
-            *(uint32_t *)&stcBconrTmp &= 0xFFFF0000ul;
+            stcBconrTmpRaw &= 0xFFFF0000ul;
         }
         else
         {
-            *(uint32_t *)&stcBconrTmp &= 0x0000FFFFul;
+            stcBconrTmpRaw &= 0x0000FFFFul;
         }
-        pstcTim0Reg->BCONR_f = stcBconrTmp;
+        pstcTim0Reg->BCONR = stcBconrTmpRaw;
         AsyncDelay(pstcTim0Reg, enCh, Enable);
-        while(*(uint32_t *)&stcBconrTmp != *(uint32_t *)&(pstcTim0Reg->BCONR_f))
+        while(stcBconrTmpRaw != pstcTim0Reg->BCONR)
         {
             if(u32TimeOut++ > TIMER0_TMOUT)
             {
@@ -686,6 +687,7 @@ en_result_t TIMER0_BaseInit(M4_TMR0_TypeDef* pstcTim0Reg,en_tim0_channel_t enCh,
                 break;
             }
         }
+        stcBconrTmp = pstcTim0Reg->BCONR_f;

         switch(enCh)
```

## `/cores/ddl/library/inc/hc32f460_pwc.h` and `/cores/ddl/library/src/hc32f460_pwc.c`

in the pwc ddl, the enum `en_pwc_stop_flash_sel` uses fairly generic names for the values, which may conflict with other code (noticably Marlin).
To mitigate this, the enum values are prefixed with `Flash` to make them more unique.

```cpp
typedef enum en_pwc_stop_flash_sel
{
    FlashWait                            = 0u,  ///< wait flash stable.
    FlashNotWait                         = 1u,  ///< Not Wait flash stable.
}en_pwc_stop_flash_sel_t;
```

This change also has to be carried to `hc32f460_pwc.c`, as the enum is used in a macro there.

```cpp
/*! Parameter validity check for flash mode while stop mode mode. */
#define IS_PWC_STOP_MODE_FLASH(x)                                              \
(   ((x) == FlashWait)                       ||                                \
    ((x) == FlashNotWait))
```

## `/cores/ddl/library/inc/hc32f460_can.h`

in the can ddl, there is a multi-line comment in the struct `stc_can_init_config` with a erreous `\` at the end of the first comment line for the field `pstcFilter`.
This causes a warning when compiling with gcc and should be removed.

```cpp
typedef struct stc_can_init_config
{
    ...
    stc_can_filter_t        *pstcFilter;        ///< @ref stc_can_filter_t Pointer to a stc_can_filter_t type array that
                                                ///<  contains the configuration informations of the acceptance filters.
    ...
}stc_can_init_config_t;
```
