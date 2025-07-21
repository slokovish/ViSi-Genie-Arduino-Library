#include "genieSTM32.h"
#ifdef STM32_HAL
#include "stm32f1xx_hal.h"
#else
#include <unistd.h> /* for sleep on host */
#endif

/* Example usage showing how to hide Keyboard 7 then show it again */

/* UART handle configured elsewhere */
UART_HandleTypeDef huart1;

int main(void)
{
    GenieSTM32 genie;

    /* Initialize the Genie library with the UART handle */
    genieSTM32_Begin(&genie, &huart1);

    /* Hide Keyboard object with index 7 */
    genieSTM32_WriteObject(&genie, GENIE_OBJ_KEYBOARD, 7, 0);

#ifdef STM32_HAL
    HAL_Delay(1000);
#else
    sleep(1);
#endif

    /* Show Keyboard object with index 7 */
    genieSTM32_WriteObject(&genie, GENIE_OBJ_KEYBOARD, 7, 1);

    /* Process events forever */
    for (;;) {
        genieSTM32_DoEvents(&genie, 1);
    }

    return 0;
}
