/**
* \brief Main source file for the I2C-Master project.
* \author Michele Grimaldi
* \date , 2020
*/
#include "InterruptRoutines.h"
#include "I2C_Interface.h"
#include "project.h"
#include "stdio.h"


/**
*   \brief 7-bit I2C address of the slave device.
*/
#define LIS3DH_DEVICE_ADDRESS 0x18

/**
*   \brief Address of the WHO AM I register
*/
#define LIS3DH_WHO_AM_I_REG_ADDR 0x0F

/**
*   \brief Address of the Status register
*/
#define LIS3DH_STATUS_REG 0x27

/**
*   \brief Address of the Control register 1
*/
#define LIS3DH_CTRL_REG1 0x20

/**
*   \brief Hex value to set normal mode and data rate selection (100Hz) to the accelerator
*/
#define LIS3DH_NORMAL_MODE_CTRL_REG1 0x57

/**
*   \brief  Address of the Temperature Sensor Configuration register
*/
#define LIS3DH_TEMP_CFG_REG 0x1F

#define LIS3DH_TEMP_CFG_REG_ACTIVE 0xC0

/**
*   \brief Address of the Control register 4: FS set to +-2g, BDU active and HR disabled
*/
#define LIS3DH_CTRL_REG4 0x23

#define LIS3DH_CTRL_REG4_BDU_ACTIVE 0x80

/**
*   \brief Address of the ADC output LSB register
*/
#define LIS3DH_OUT_ADC_3L 0x0C

/**
*   \brief Address of the ADC output MSB register
*/
#define LIS3DH_OUT_ADC_3H 0x0D

/**
*   \brief Address of the Acceletation X Output LSB register (28h)
*/
#define LIS3DH_OUT_X_L 0x28


int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    I2C_Peripheral_Start();
    UART_Debug_Start();

    CyDelay(5); //"The boot procedure is complete about 5 milliseconds after device power-up."

    // String to print out messages on the UART
    char message[50];

    // Check which devices are present on the I2C bus
    for (int i = 0 ; i < 128; i++)
    {
        if (I2C_Peripheral_IsDeviceConnected(i))
        {
            // print out the address is hex format
            sprintf(message, "Device 0x%02X is connected\r\n", i);
            UART_Debug_PutString(message);
        }

    }
    /******************************************/
    /*            I2C Writing                 */
    /******************************************/

    uint8_t ctrl_reg1;

    ctrl_reg1 = LIS3DH_NORMAL_MODE_CTRL_REG1;

    ErrorCode error = I2C_Peripheral_WriteRegister(LIS3DH_DEVICE_ADDRESS,
                                             LIS3DH_CTRL_REG1,
                                             ctrl_reg1);

    if (error == ERROR)
    {
        UART_Debug_PutString("Error occurred during I2C comm to set control register 1\r\n");
    }

    uint8_t tmp_cfg_reg = LIS3DH_TEMP_CFG_REG_ACTIVE; // tmp_cfg_reg must be changed to the appropriate value

    error = I2C_Peripheral_WriteRegister(LIS3DH_DEVICE_ADDRESS,
                                         LIS3DH_TEMP_CFG_REG,
                                         tmp_cfg_reg);
    if (error == ERROR)
    {
        UART_Debug_PutString("Error occurred during I2C comm to write TEMP_CFG_REG\r\n");
    }
    uint8_t ctrl_reg4;

    ctrl_reg4 = LIS3DH_CTRL_REG4_BDU_ACTIVE; // ctrl_reg4 must be changed to the appropriate value

    error = I2C_Peripheral_WriteRegister(LIS3DH_DEVICE_ADDRESS,
                                         LIS3DH_CTRL_REG4,
                                         ctrl_reg4);

    if (error == ERROR)
    {
        UART_Debug_PutString("Error occurred during I2C comm to read control register4\r\n");
    }
    
}
/* [] END OF FILE */
