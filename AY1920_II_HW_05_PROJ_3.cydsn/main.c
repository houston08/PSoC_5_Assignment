/**
* \brief Main source file for the I2C-Master project.
* \author Michele Grimaldi
* \date , 2020
*/
#include "InterruptRoutines.h"
// Include required header files
#include "I2C_Interface.h"
#include "project.h"
#include "stdio.h"
#include <math.h>


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
*   \brief Hex value to set 100Hz as Data Rate and LPen set to 0 
*/
#define LIS3DH_NORMAL_MODE_CTRL_REG1 0x57

/**
*   \brief Address of the Control register 4
*/
#define LIS3DH_CTRL_REG4 0x23

#define LIS3DH_CTRL_REG4_BDU_HR_4G_ACTIVE 0x98 //HR active; BDU active + FS a 01 (+-4g)

/**
*   \brief Address of the Acceletation X Output LSB register (28h)
*/
#define LIS3DH_OUT_X_L 0x28

_VOLATILE  uint8 flag_ready0=0; //Volatile Global variable enabling to sample

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
    /*******************************************/
    /*I2C Writing of registers at proper values*/ 
    /*******************************************/

    uint8_t ctrl_reg1;

    ctrl_reg1 = LIS3DH_NORMAL_MODE_CTRL_REG1; // ctrl_reg1 must be changed to the appropriate value

    ErrorCode error = I2C_Peripheral_WriteRegister(LIS3DH_DEVICE_ADDRESS,
                                             LIS3DH_CTRL_REG1,
                                             ctrl_reg1);

    if (error == ERROR)
    {
        UART_Debug_PutString("Error occurred during I2C comm to set control register 1\r\n");
    }

    
    uint8_t ctrl_reg4;

    ctrl_reg4 = LIS3DH_CTRL_REG4_BDU_HR_4G_ACTIVE; // ctrl_reg4 must be changed to the appropriate value

    error = I2C_Peripheral_WriteRegister(LIS3DH_DEVICE_ADDRESS,
                                         LIS3DH_CTRL_REG4,
                                         ctrl_reg4);

    if (error == ERROR)
    {
        UART_Debug_PutString("Error occurred during I2C comm to read control register4\r\n");
    }
    
    /*VARIABLE DECLARATION*/
    
    /* Flags used to perform an accurate sampling of 100Hz frequency */
    uint8_t flag_ready1=0; 
    uint8_t flag_ready2=0;
    
    uint8_t status_register; //Variable that will store the status register value
    uint8_t register_count=6; //Numebers of byte to be read to get the 3 axis accelerartion
    
    /*Acceleration value in digit will be stored in 3 16 bit integers*/
    int16_t OutX;
    int16_t OutY;
    int16_t OutZ;
    /*Acceleration value in m/s^2 will be stored in 3 32bit float variables */
    float32 AccX;
    float32 AccY;
    float32 AccZ;
    
    /*Acceleration float value in m/s^2 will be cast in 3 32bit int variables */
    
    int32_t AccXint;
    int32_t AccYint;
    int32_t AccZint;
    
    /*uint8_t Vector whose size is equal to the number of bytes to be read to obtain 
    acceleration value: 3 axis(X,Y,Z) times 2 byte (LSB ans MSB) */
    uint8_t AccData[6];   
    
    /*UART Transmission */
    
    uint8_t header = 0xA0; //Header
    uint8_t footer = 0xC0; //Footer
    uint8_t OutArray[14]; //Array of uint8_t that will be sent through UART.
    //Its dimension is 4bytes*Number_of_axis(3)+ 2bytes(Header and Footer)=14
    
    OutArray[0] = header; 
    OutArray[13] = footer;
    
    Timer_Start(); // Timer initialization
    isr_TIMER_StartEx(Custom_TIMER_ISR); //Interrupt pointing to the correct function address
    
    for(;;)
    {
        if(flag_ready0) 
        {
            /*I read the status register and evaluate if new data is avaible 
                checking the value of ZYXDA bit. flag_ready1 will be 0 if 
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
                                                              register_count,
                                                                &AccData[0]);
                if(error == NO_ERROR)
                {
                    /*In HR mode the output data are 12 left justified bit so
                    to get the output I firstly do an or operation between LSB
                    and MSB shitfed of eight position towards left, then to 
                    achieve the right justification a shift of 4 bit towards 
                    right is operated. The result is casted to 3 int16 variables */
                    
                    OutX = (int16)((AccData[0] | (AccData[1]<<8)))>>4;
                    OutY = (int16)((AccData[2] | (AccData[3]<<8)))>>4;
                    OutZ = (int16)((AccData[4] | (AccData[5]<<8)))>>4;
                    
                    /* We want to get the acceleration value in m/s^2 starting from its 
                    digit value. The latter multiplied by the sensor sensitivity (2mg/digit if
                    FS bit set to 01) gives the acceleration value in mg. Then multiplying by 
                    the earth gravitational acceleration constant g (9.80665 m/s^2) and dividing
                    by 1000 gives us the acceleration value in m/s^2. AccX,AccY and AccZ are
                    float32 variables.*/
                    
                    AccX=(OutX*9.80665*2)/1000; 
                    AccY=(OutY*9.80665*2)/1000;
                    AccZ=(OutZ*9.80665*2)/1000;
                    /* In order to cast a float value to a int value I can multiply the float value
                    i.e. by 1000 keeping also 3 decimals information (and not only the integer value).
                    Since the int value will be sent to Bridge control panel we have to set there 
                    the proper scale value (0.001) for the 3 axis acceleration to get again the m/s^2 
                    values. I add 0.5 to perform a round of the third decimal (the cast to int floors the 
                    value to the previous integer e.g. 3.9->3, while in this way to the nearest integer 
                    3.9->4)
                    Alternativly, if we don't want to loose any information from float variable,
                    we could have done a cast as follow:
                    uint8_t *pointer1=(uint8_t*)&AccX;
                    uint8_t *pointer2=(uint8_t*)&AccY;
                    uint8_t *pointer3=(uint8_t*)&AccZ;
                    Then we would have sent through UART pointer1[0](LSB of AccX)...pointer[3](MSB of AccX),
                    pointer2[0](LSB of AccY)...[3](MSB of AccY), pointer3[0](LSB of AccZ)...[3](MSB of AccZ)
                    that contain the 4 bytes of the 3 float variables and set in Bridge Control Panel 3 float 
                    signed variables.
                    */
                    AccXint=(int32)(AccX*1000+0.5);
                    AccYint=(int32)(AccY*1000+0.5);
                    AccZint=(int32)(AccZ*1000+0.5);
                    
                    /*Values send through UART. Each byte is computed through a uint8_T cast
                    and & operation and shift identifying the proper byte from LSB to MSB*/
                    //X acceleration bytes value 
                    OutArray[1] = (uint8_t)(AccXint & 0xFF);
                    OutArray[2] = (uint8_t)(AccXint >> 8);
                    OutArray[3] = (uint8_t)(AccXint >> 16);
                    OutArray[4] = (uint8_t)(AccXint >> 24);
                    //Y acceleration bytes value 
                    OutArray[5] = (uint8_t)(AccYint & 0xFF);
                    OutArray[6] = (uint8_t)(AccYint >> 8);
                    OutArray[7] = (uint8_t)(AccYint >> 16);
                    OutArray[8] = (uint8_t)(AccYint >> 24);
                    //Z acceleration bytes value
                    OutArray[9] = (uint8_t)(AccZint & 0xFF);
                    OutArray[10] = (uint8_t)(AccZint >> 8);
                    OutArray[11] = (uint8_t)(AccZint >> 16);
                    OutArray[12] = (uint8_t)(AccZint >> 24);                    
                    
                    UART_Debug_PutArray(OutArray, 14); //Array sent through UART
                    flag_ready0=0; // flag_ready will return to 1 the next time an isr will be generated
                }
            }
        }
    }
              
}
    

/* [] END OF FILE */
