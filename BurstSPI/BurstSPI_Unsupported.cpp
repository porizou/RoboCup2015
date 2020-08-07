#if !(defined(TARGET_KL25Z) || defined(TARGET_KL46Z))
#if !(defined(TARGET_LPC1768) || defined(TARGET_LPC1114) || defined(TARGET_LPC11U24) || defined(TARGET_LPC13XX))
#if !(defined(TARGET_NUCLEO_L152RE))

#warning BurstSPI target not supported, reverting to regular SPI

#include "BurstSPI.h"

void BurstSPI::fastWrite(int data) {
    write(data);
}

void BurstSPI::clearRX( void ) {

}
#endif          //Freescale
#endif          //NXP
#endif          //NUCLEO