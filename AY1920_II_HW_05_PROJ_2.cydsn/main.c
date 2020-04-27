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
    
        uint8_t register_count_temp=2;//I have to read 2(LSB and MSB) byte
    uint8_t register_count_acc=6; //I have to read 2(LSB and MSB)*3(X,Y,Z) byte
    int16_t OutX;
    int16_t OutY;
    int16_t OutZ;
    int16_t OutTemp;
    uint8_t header = 0xA0;
    uint8_t footer = 0xC0;
    uint8_t OutArray[8];
    uint8_t TemperatureData[2];
    uint8_t AccData[6];
    uint8_t flag_ready1=0;
    uint8_t flag_ready2=0;
    uint8_t status_register;
    OutArray[0] = header;
    OutArray[7] = footer;
    /*I read the Temperature that will be used later to compute the sensor sensitivity change.
      Since the sensitivity change is small and assuming no abrupt temperature changes after
      device power on the temperature value can be read just one time.*/
    error = I2C_Peripheral_ReadRegisterMulti(LIS3DH_DEVICE_ADDRESS,
                                                      LIS3DH_OUT_ADC_3L,
                                                      register_count_temp,
                                                      &TemperatureData[0]);
    OutTemp = (int16)((TemperatureData[0] | (TemperatureData[1]<<8)))>>6;
    Timer_Start();
    isr_TIMER_StartEx(Custom_TIMER_ISR);
    
    //I read the Temperature that will be used later to compute the sensor sensitivity change
    
    for(;;)
    {
        if(flag_ready0)
        {
            /*I read the status register and evaluate if new data is avaible 
                checking the value of ZYXDA bit. Flag register will be 0 if 
                no new data are avaiable while will be greater than zero(value = 8)
                if new data are avaiable;
                The cycle continues untill flag_ready1 is different from zero. Then if 
                ZYXOR bit is equal to 1 it means a new set of data has overwritten the previous set,
                so I can now read the accelerometer outputs
            */
            while(!flag_ready1)
            {
                error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                            LIS3DH_STATUS_REG,
                                            &status_register);
                if(error == NO_ERROR)
                    flag_ready1=(status_register&(1<<3)); // & operation to check the value of ZYXDA bit
            }
            flag_ready2=(status_register&(1<<7)); // & operation to check the value of ZYXOR bit.
            if(flag_ready2)
            {
                /*I can now read the accelerometer output through the multiread function starting
                from LIS3DH_OUT_X_L(28h) to OUT_Z_H (2Dh) saving values is AccData */
                error = I2C_Peripheral_ReadRegisterMulti(LIS3DH_DEVICE_ADDRESS,
                                                              LIS3DH_OUT_X_L,
                                                              register_count_acc,
                                                                &AccData[0]);
                if(error == NO_ERROR)
                {
                    //to do
                    }
                flag_ready0=0;
            }
        }
    }
}
/* [] END OF FILE */
