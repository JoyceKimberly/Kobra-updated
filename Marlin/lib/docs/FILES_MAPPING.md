# Files Mapping

the ddl files provided by HDSC (hc32f460_ddl_Revx.y.z.zip) are mapped to the files in this core as follows:

| core file                | ddl package file                                                       |
| ------------------------ | ---------------------------------------------------------------------- |
| `cores/ddl/addon/*`      | -                                                                      |
| `cores/ddl/cmsis/*`      | `mcu/GCC/CMSIS/Core/Include/*`                                         |
| `cores/ddl/common/*`     | `mcu/common/*`                                                         |
| `cores/ddl/compat/*`     | -                                                                      |
| `cores/ddl/library/*`    | `driver/*`                                                             |
| `middleware/*`           | `midware/*`                                                            |
| `cores/ddl/startup/*`    | `mcu/GCC/Startup/*`                                                    |
| `cores/ddl/ddl_config.h` | based on `example/ev_hc32f460_lqfp100_v2/template/source/ddl_config.h` |
| `cores/ddl/version.txt`  | `version.txt`                                                          |
| `ld/hc32f46x_param.ld`   | parameterized version of `mcu/GCC/LinkerScript/hc32f460xc_flash.ld`    |
