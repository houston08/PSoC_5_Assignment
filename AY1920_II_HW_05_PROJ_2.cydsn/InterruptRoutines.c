/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include "InterruptRoutines.h"
/* Since we want to sample the sensor output at a frequency 
   of 100Hz, the timer is set to generate an Interrupt service
   routine every 10ms. The isr will change the value of volatile
   flag_ready0 variable to 1 leading to the reading of the sensor
   output in main function.     */

_VOLATILE  uint8 flag_ready0; // Volatile Global variable enabling to sample

CY_ISR(Custom_TIMER_ISR)
{
    Timer_ReadStatusRegister(); // Timer Reset 
    flag_ready0=1;
}
/* [] END OF FILE */
