#include "led_error.h"







/* Public Functions -------------------------------------------------------------------------------*/\
void LED_Error_Handler(LED_ErrorState error) {
    /* If do not have any error. */
    if(error == ERROR_NONE) {
        Turn_On_Green_Led();
        Turn_Off_Red_Led();
    } 

    /*If error show-up. */
    else {
        Turn_Off_Red_Led();
        Turn_Off_Green_Led();

        for(uint8_t i=0; i<error; i++) {
            Turn_On_Red_Led();
            HAL_Delay(300);
            Turn_Off_Red_Led();
            HAL_Delay(300);
        }
        HAL_Delay(2000);
    }
}


