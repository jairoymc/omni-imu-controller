/******************************************************************************
 * Mech Final Project
 * Author: Jairo Maldonado
 *
 * MPU6050 Registers (16 bit per axis, 2 bytes each register)
 * MSB + LSB
 * ax = 0x3B + 0x3C
 * ay = 0x3D + 0x3E
 * az = 0x3F + 0x40
 * gx = 0x43 + 0x44
 * gy = 0x45 + 0x46
 * gz = 0x47 + 0x48
 *
 *CONNECTIONS
 * MPU6050  MSP
 * VCC ----> 5V
 * GND ----> GND
 * SCL ----> 1.7
 * SDA ----> 1.6
 *
*******************************************************************************/
#include "driverlib.h"
#include "main.h"
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E
#define WOM_THR          0x1F

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E


#define MPU9250_ADDRESS   0x68

#define sampleFreq  99.95f      // sample frequency in Hz
#define betaDef     0.1f        // 2 * proportional gain
#define M_PI    3.14159
#define degreesToRadians(angleDegrees) ((angleDegrees) * M_PI / 180.0)
#define radiansToDegrees(angleRadians) ((angleRadians) * 180.0 / M_PI)
#define ABS(X)  ((X>=0)? X : -(x) )
#define ROUND(X)  (X>=0)? (int) (X + 0.5) : (int)-(ABS(X) +0.5)

// IMU conversion factors
float acc_factor = 16384.0;
float gyro_factor = 131.0;

// IMU vars
volatile float ax, ay, az, gx, gy, gz;
volatile float yaw_gyro = 0.0f;
volatile float mag;
volatile int heading;



int i = 0;
uint8_t axReg[] = {ACCEL_XOUT_H,ACCEL_XOUT_L};
uint8_t ayReg[] = {ACCEL_YOUT_H,ACCEL_YOUT_L};
uint8_t azReg[] = {ACCEL_ZOUT_H,ACCEL_ZOUT_L};
uint8_t gxReg[] = {GYRO_XOUT_H,GYRO_XOUT_L};
uint8_t gyReg[] = {GYRO_YOUT_H,GYRO_YOUT_L};
uint8_t gzReg[] = {GYRO_ZOUT_H,GYRO_ZOUT_L};

const eUSCI_I2C_MasterConfig i2cConfig =
{
EUSCI_B_I2C_CLOCKSOURCE_SMCLK, // SMCLK Clock Source
3000000, // SMCLK = 3MHz
EUSCI_B_I2C_SET_DATA_RATE_400KBPS, // Desired I2C Clock of 400khz
0, // No byte counter threshold
EUSCI_B_I2C_NO_AUTO_STOP // No Autostop
};

const Timer_A_UpModeConfig TimerConfig = // Configure counter in Up mode. DCO/[(DCO:SMLCK DIVIDER)*(TIMERA Period)]=1hz
{
TIMER_A_CLOCKSOURCE_SMCLK,      // Tie Timer A to SMCLK
TIMER_A_CLOCKSOURCE_DIVIDER_64, // Increment counter every 64 clock cycles
469,// Period of Timer A
TIMER_A_TAIE_INTERRUPT_DISABLE, // Disable Timer A rollover interrupt64
TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE, // Enable Capture Compare interrupt
TIMER_A_DO_CLEAR // Clear counter upon initialization
};

//-------------------------------
//Tilt Sensing
volatile float accel_angle_x = 0;
volatile float accel_angle_y = 0;

//---------------------------------
//Function Prototypes
int MPUread(uint8_t *Register);
float invSqrt(float x);
void IMUcalibrate(void);
void calc_roll_pitch_angles(void);
void cmd_vel(void);
void getYaw_gyro(void);
void MPU9250_setup(void);
int readSingleByte(uint8_t Reg);
int readDoubleByte(uint8_t *Reg);
//--------------------------------
//Calibration
volatile float ax_sum = 0.0;
volatile float ay_sum = 0.0;
volatile float az_sum = 0.0;
volatile float gx_sum = 0.0;
volatile float gy_sum = 0.0;
volatile float gz_sum = 0.0;

volatile float ax_bias = 0.0;
volatile float ay_bias = 0.0;
volatile float az_bias = 0.0;
volatile float gx_bias = 0.0;
volatile float gy_bias = 0.0;
volatile float gz_bias = 0.0;

int count = 0; // Increment to calibrate imu
int count_done = 10; // Collect this amount of IMU data to average
float data1[100];
float data2[100];
float data3[100];
volatile int count_data = 0;
volatile int x;
int print = 0;
volatile int index = 0;
volatile float ang_vel = 0;

int main(void)
{
    MAP_WDT_A_holdTimer();

    unsigned int dcoFrequency = 3E+6;
    MAP_CS_setDCOFrequency(dcoFrequency); //Set DCO clock source frequency
    MAP_CS_initClockSignal(CS_SMCLK,CS_DCOCLK_SELECT,CS_CLOCK_DIVIDER_1); //Tie SMCLK to DCO

    //Configure and Enable Timer A interrupt. Interrupt every second.
    MAP_Timer_A_configureUpMode(TIMER_A0_BASE,&TimerConfig);  // Configure Timer A using above struct
    MAP_Interrupt_enableInterrupt(INT_TA0_0);   // Enable Timer A interrupt

    //Led for debugging
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN0); // LED1 Output
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0); // LED1 LOW
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2,GPIO_PIN0); //LED2 RED Output
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0); //LED2 RED Low
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2,GPIO_PIN1); //LED2 RED Output
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1); //LED2 RED Low

    //Set up SDA and SCL pins
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION); //SDA
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION); //SCL

    // Initialize 12C
    MAP_I2C_initMaster(EUSCI_B1_BASE, &i2cConfig); /* Initializing I2C Master to SMCLK at 100kbs with no autostop */
    MAP_I2C_setSlaveAddress(EUSCI_B1_BASE, MPU9250_ADDRESS); /* Specify slave address */
    MAP_I2C_enableModule(EUSCI_B1_BASE); /* Enable I2C Module to start operations */
    MAP_I2C_setMode(EUSCI_B1_BASE, EUSCI_B_I2C_TRANSMIT_MODE); /* Set Master in transmit mode */

    MPU9250_setup();

    //MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);  // Start Timer A
    MAP_Interrupt_enableMaster(); // Enable all interrupts
    IMUcalibrate();
    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);  // Start Timer A
    while(1){
            if (count_data > 2000 & print==0){ // number of data samples
                int i;
                for(i = 0; i < 80; i++){ //collected data
                    printf("%.3f\n",data3[i]);
                }
                print = 1;
            }
        }
}


//Get an IMU Reading every second
void TA0_0_IRQHandler(void){
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0); //Clear CC interrupt
    // Used during calibration
    if (count < count_done){
        ax = MPUread(axReg)/acc_factor; //Gs
        ay = MPUread(ayReg)/acc_factor;
        az = MPUread(azReg)/acc_factor;
        gx = MPUread(gxReg)/gyro_factor; //degrees/sec
        gy = MPUread(gyReg)/gyro_factor;
        gz = MPUread(gzReg)/gyro_factor;

        ax_sum = ax_sum + ax;
        ay_sum = ay_sum + ay;
        az_sum = az_sum + az;
        gx_sum = gx_sum + gx;
        gy_sum = gy_sum + gy;
        gz_sum = gz_sum + gz;
        count++;
        if(count == count_done){
            printf("Done\n");
            MAP_Timer_A_stopTimer(TIMER_A0_BASE);
        }
    }
    else if(count == count_done+1){
        //for magwick switch ax and ay AND gx and gy. And multiply by 1000.
        ax = (MPUread(axReg)/acc_factor - ax_bias); //Gs
        ay = (MPUread(ayReg)/acc_factor - ay_bias);
        az = (MPUread(azReg)/acc_factor - az_bias);
        gx = (MPUread(gxReg)/gyro_factor - gx_bias); //degrees/sec
        gy = (MPUread(gyReg)/gyro_factor - gy_bias);
        gz = (MPUread(gzReg)/gyro_factor - gz_bias);

//        printf("ax: %.3f  ay: %.3f  az: %.3f\n",ax,ay,az);
//        printf("gx: %.3f  gy: %.3f  gz: %.3f\n",gx,gy,gz);
//        printf("mx: %f  my: %f  mz: %f\n",mx,my,mz);
//        printf("\n");

        getYaw_gyro(); //yaw_gyro
        calc_roll_pitch_angles(); // roll and pitch
        cmd_vel(); // heading and magnitude
//        printf("yaw: %.3f\n",yaw_gyro);
//        printf("\n");

        if (count_data%25 == 0){
//            printf("mag: %.3f\n",mag);
//            printf("heading: %d\n",heading);
//            printf("yaw: %.3f\n",yaw_gyro);
//            printf("\n");

            data3[in`dex] = ang_vel;
            index++;
        }
        count_data++;
        if (count_data > 2000){
            Timer_A_stopTimer(TIMER_A0_BASE);
        }
    }
}

void IMUcalibrate(void){
    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);  // Start Timer A
    while (count < count_done){}
    printf("calculating bias\n");
    ax_bias = ax_sum/count_done;
    ay_bias = ay_sum/count_done;
    az_bias = az_sum/count_done - 1;
    gx_bias = gx_sum/count_done;
    gy_bias = gy_sum/count_done;
    gz_bias = gz_sum/count_done;
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
    count++;
}

int MPUread(uint8_t *Reg)
{
    int val;
    MAP_I2C_setSlaveAddress(EUSCI_B1_BASE, MPU9250_ADDRESS); /* Specify slave address */
    val = readDoubleByte(Reg);
    return val;
}

void MPU9250_setup(void){
    //MPU9250 Setup
    MAP_I2C_setSlaveAddress(EUSCI_B1_BASE, MPU9250_ADDRESS); /* Specify slave address */
    // Wake up IMU via PWR_MGMT_1
    MAP_I2C_masterSendMultiByteStart(EUSCI_B1_BASE,PWR_MGMT_1);
    MAP_I2C_masterSendMultiByteFinish(EUSCI_B1_BASE, 0x00);
}

int readSingleByte(uint8_t Reg){
    int RXData;
    MAP_I2C_setMode(EUSCI_B1_BASE, EUSCI_B_I2C_TRANSMIT_MODE); /* Set Master in transmit mode */
    MAP_I2C_masterSendSingleByte(EUSCI_B1_BASE, Reg); //Send start, transmits byte to slave, send stop
    while(MAP_I2C_masterIsStopSent(EUSCI_B1_BASE)){}
    MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P2,GPIO_PIN0); //Toggle LED1
    MAP_I2C_setMode(EUSCI_B1_BASE, EUSCI_B_I2C_RECEIVE_MODE); /* Set Master in transmit mode */
    MAP_I2C_masterReceiveStart(EUSCI_B1_BASE); // Get ready to read
    RXData = MAP_I2C_masterReceiveMultiByteFinish(EUSCI_B1_BASE); // Read one byte
    return RXData;
}

int readDoubleByte(uint8_t *Reg){
    int i = 0;
    uint8_t RXData[2];
    int16_t Data;

    for (i=0; i<2;i++){
        MAP_I2C_setMode(EUSCI_B1_BASE, EUSCI_B_I2C_TRANSMIT_MODE); /* Set Master in transmit mode */
        MAP_I2C_masterSendSingleByte(EUSCI_B1_BASE, Reg[i]); //Send start, transmits byte to slave, send stop
        while(MAP_I2C_masterIsStopSent(EUSCI_B1_BASE)){}
        MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P2,GPIO_PIN0); //Toggle LED1
        MAP_I2C_setMode(EUSCI_B1_BASE, EUSCI_B_I2C_RECEIVE_MODE); /* Set Master in transmit mode */
        MAP_I2C_masterReceiveStart(EUSCI_B1_BASE); // Get ready to read
        RXData[i] = MAP_I2C_masterReceiveMultiByteFinish(EUSCI_B1_BASE); // Read one byte
    }
    Data = (int16_t)(RXData[0] << 8 | RXData[1]);
    return Data;
}

float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void calc_roll_pitch_angles(void){
   // Using x y and z from accelerometer, calculate x and y angles
   float x_val, y_val, z_val, result;
   float x2, y2, z2; //24 bit

   x_val = (float)ax;
   y_val = (float)ay;
   z_val = (float)az;

   // Work out the squares
   x2 = (double)(x_val*x_val);
   y2 = (double)(y_val*y_val);
   z2 = (double)(z_val*z_val);

   //Y Axis Pitch
   result=sqrt(y2+z2);
   result=x_val/result;
   accel_angle_y = -radiansToDegrees(atan(result));

   //X Axis Roll
   result=sqrt(x2+z2);
   result=y_val/result;
   accel_angle_x = -radiansToDegrees(atan(result));

}

void cmd_vel(void){
    float dir[2];
    int angle;

    if (az > 0.990 && az < 1.010){ // Level = No direction
        mag = 0;

        angle = 999;
        heading = 999;
    }
    else{
        mag = sqrt((accel_angle_y/90.0)*(accel_angle_y/90.0) + (accel_angle_x/90.0)*(accel_angle_x/90.0)); // Magnitude of incline in percentage

        dir[0] = degreesToRadians(accel_angle_y/90.0); // X axis
        dir[1] = degreesToRadians(accel_angle_x/90.0); // Y axis
        angle = (int) (radiansToDegrees(atan2(dir[1],dir[0])));

        // Heading 0->359
        if (angle >= -179 && angle <= -1){ // heading between 91-269
            heading = 90 - angle;
        }
        else if(angle >= 91 && angle <= 180){ // heading between 270-359
            heading = 450 - angle;
        }
        else{ //heading between 0-90
            heading = 90 - angle;
        }

        // Heading 0->180 and 0->-179
//        if (angle >= -89 && angle <= 180){
//            heading = angle - 90;
//        }
//        else{
//            heading = angle + 270;
//        }
    }
}

void getYaw_gyro(void){
    yaw_gyro = yaw_gyro - gz*(1/sampleFreq);
    if (yaw_gyro < 10 & yaw_gyro > -10){
        ang_vel = 0;
    }
    else if(yaw_gyro > 90){
        ang_vel = 1;
    }
    else if(yaw_gyro < -90){
        ang_vel = -1;
    }
    else{
        ang_vel = yaw_gyro/90;
    }
}
