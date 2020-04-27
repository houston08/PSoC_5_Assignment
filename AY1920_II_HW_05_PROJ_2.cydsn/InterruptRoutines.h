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
#ifndef _INTERRUPT_ROUTINES_H_
    // Header guard
    #define _INTERRUPT_ROUTINES_H_
    
    #include "project.h"
    
    /**
    *   \brief ISR Code.
    */
    CY_ISR_PROTO(Custom_TIMER_ISR);
    _VOLATILE  uint8 flag_ready0; // Global variable that keeps memory of the led blinkin pattern state (value from 1 to 7)

#endif
/* [] END OF FILE */
