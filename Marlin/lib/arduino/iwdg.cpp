#include "iwdg.h"
#include "wdt.h"


void iwdg_init(void)
{
    bsp_wdt_init();
    bsp_wdt_refresh();
}

void iwdg_feed(void)
{
    bsp_wdt_refresh();
}

