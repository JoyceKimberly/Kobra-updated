#ifdef TARGET_HC32F46x
#include "sdio.h"
#include <gpio/gpio.h>
#include <sd_card.h>

#define SDIOC_CD_PORT                   (PortA)
#define SDIOC_CD_PIN                    (Pin10)

#define SDIOC_D0_PORT                   (PortC)
#define SDIOC_D0_PIN                    (Pin08)

#define SDIOC_D1_PORT                   (PortC)
#define SDIOC_D1_PIN                    (Pin09)

#define SDIOC_D2_PORT                   (PortC)
#define SDIOC_D2_PIN                    (Pin10)

#define SDIOC_D3_PORT                   (PortC)
#define SDIOC_D3_PIN                    (Pin11)

#define SDIOC_CK_PORT                   (PortC)
#define SDIOC_CK_PIN                    (Pin12)

#define SDIOC_CMD_PORT                  (PortD)
#define SDIOC_CMD_PIN                   (Pin02)

#define SDIO_INTERFACE M4_SDIOC1

#define SDIO_READ_RETRIES 3
#define SDIO_WRITE_RETRIES 1
#define SDIO_TIMEOUT 100u

#define WITH_RETRY(retries, fn)                     \
	for (int retry = 0; retry < (retries); retry++) \
	{                                               \
		MarlinHAL::watchdog_refresh();              \
		fn                                          \
	}

static stc_sd_handle_t cardHandle;

const stc_sdcard_dma_init_t dmaConf = {
	.DMAx = M4_DMA1,
	.enDmaCh = DmaCh0,
};

const stc_sdcard_init_t cardConf = {
	.enBusWidth = SdiocBusWidth4Bit,
	.enClkFreq = (en_sdioc_clk_freq_t)16000000u,
	.enSpeedMode = SdiocHighSpeedMode,
	.pstcInitCfg = NULL,
};

bool SDIO_Init()
{
	// configure SDIO pins
    PORT_SetFunc(SDIOC_D0_PORT, SDIOC_D0_PIN, Func_Sdio, Disable);
    PORT_SetFunc(SDIOC_D1_PORT, SDIOC_D1_PIN, Func_Sdio, Disable);
    PORT_SetFunc(SDIOC_D2_PORT, SDIOC_D2_PIN, Func_Sdio, Disable);
    PORT_SetFunc(SDIOC_D3_PORT, SDIOC_D3_PIN, Func_Sdio, Disable);
    PORT_SetFunc(SDIOC_CD_PORT, SDIOC_CD_PIN, Func_Sdio, Disable);
    PORT_SetFunc(SDIOC_CK_PORT, SDIOC_CK_PIN, Func_Sdio, Disable);
    PORT_SetFunc(SDIOC_CMD_PORT, SDIOC_CMD_PIN, Func_Sdio, Disable);

	// create sdio handle
	MEM_ZERO_STRUCT(cardHandle);
	cardHandle.SDIOCx = SDIO_INTERFACE;
	cardHandle.enDevMode = SdCardDmaMode;
	cardHandle.pstcDmaInitCfg = &dmaConf;

	// initialize sd card
	en_result_t rc = SDCARD_Init(&cardHandle, &cardConf);
	if (rc != Ok)
	{
		printf("SDIO_Init() error (rc=%u)\n", rc);
	}

	return rc == Ok;
}

bool SDIO_ReadBlock(uint32_t block, uint8_t *dst)
{
	WITH_RETRY(SDIO_READ_RETRIES, {
		en_result_t rc = SDCARD_ReadBlocks(&cardHandle, block, 1, dst, SDIO_TIMEOUT);
		if (rc == Ok)
		{
			return true;
		}
		else
		{
			printf("SDIO_ReadBlock error (rc=%u)\n", rc);
		}
	})

	return false;
}

bool SDIO_WriteBlock(uint32_t block, const uint8_t *src)
{
	WITH_RETRY(SDIO_WRITE_RETRIES, {
		en_result_t rc = SDCARD_WriteBlocks(&cardHandle, block, 1, (uint8_t *)src, SDIO_TIMEOUT);
		if (rc == Ok)
		{
			return true;
		}
		else
		{
			printf("SDIO_WriteBlock error (rc=%u)\n", rc);
		}
	})

	return false;
}

bool SDIO_IsReady()
{
	return bool(cardHandle.stcCardStatus.READY_FOR_DATA);
}

uint32_t SDIO_GetCardSize()
{
	// multiply number of blocks with block size to get size in bytes
	uint64_t cardSizeBytes = uint64_t(cardHandle.stcSdCardInfo.u32LogBlockNbr) * uint64_t(cardHandle.stcSdCardInfo.u32LogBlockSize);

	// if the card is bigger than ~4Gb (maximum a 32bit integer can hold), clamp to the maximum value of a 32 bit integer
	if(cardSizeBytes >= UINT32_MAX)
	{
		return UINT32_MAX;
	}

	return uint32_t(cardSizeBytes);
}

#endif // TARGET_HC32F46x
