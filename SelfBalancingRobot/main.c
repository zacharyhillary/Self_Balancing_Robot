
//*****************************************************************************
// Zachary Hillary
// 12/17/2022
// Self Balancing Robot Project
// This project is a two wheeled robot that remains stable by reading Accelerometer/Gyroscope data from MPU6050 and adjusting motor speed and direction it also allows you to go forward/reverse/left/right via remote control using UART communication protocol
// Microprocessor: Tiva C TM4C123gxl
// IMU (Inertial Measurement Unit) MPU6050
// Motor driver L298N

//Ports used
// PB2:SCL(I2C)
// PB3:SDA(I2C)
// PC4: RX (UART)(TX ON BLUETOOTH)
// PC3 : TX (UART)(RX ON BLUETOOTH)
// PE3: KP(ADC)
// PE2: KI(ADC)
// PE1: KD(ADC)
// PE4:Left Motor Speed(PWM)
// PE5:Right Motor Speed(PWM)
// PD0 & PD1 : Right Motor Direction(Digital Output)
// PD2 & PD3 : Left Motor Direction(Digital Output)
//
//
//*****************************************************************************
#include <math.h>
#include <stdbool.h> //adds bool data type
#include <stdint.h>  //adds new integer data types
#include <stdio.h>
#include <stdlib.h>
#include "Board.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h" //Map of registers for the TI EK123gxl board
#include "utils/uartstdio.h"
#include <ti/drivers/GPIO.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <xdc/runtime/System.h>
#include <xdc/std.h>
#include "driverlib/adc.h"
#define MPU6050_ADDRESS 0x68 // device address for MPU_6050

#define PWR_MGMT_1 0x6B // register addresses for MPU_6050 I2C
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

typedef struct MPU6050_REGISTERS
{
  int address[13];
  unsigned int data[13];
} MPU6050_REGISTERS;

//Global Variables
char RemoteControl;
double desiredAngle;
double kp, ki, kd;
int kp_raw, ki_raw, kd_raw;
double CalibratedZeroAngle;
int GyroYOffset = 0;
int16_t AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ;
double lasterror = 0;
double sumError = 0;
double FilteredAngle = 0;
int turningleft = false;
int turningright = false;
int iteration=0;

MPU6050_REGISTERS MPU6050; // create new instance of a struct
void ADC_Config () // Configures ADC usage for PE3 PE2 PE1
{
  SysCtlPeripheralEnable (SYSCTL_PERIPH_ADC0);  // Enable ADC0
  SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOE); // Enable Port E
  GPIOPinTypeADC (GPIO_PORTE_BASE, GPIO_PIN_3); // Enable PE
  GPIOPinTypeADC (GPIO_PORTE_BASE, GPIO_PIN_2);
  GPIOPinTypeADC (GPIO_PORTE_BASE, GPIO_PIN_1);

  ADCSequenceDisable (ADC0_BASE, 1);
  ADCSequenceConfigure (ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
  ADCSequenceStepConfigure (ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END); // PE3

  ADCSequenceConfigure (ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
  ADCSequenceStepConfigure (ADC0_BASE, 2, 0, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END); // PE2

  ADCSequenceConfigure (ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
  ADCSequenceStepConfigure (ADC0_BASE, 1, 0, ADC_CTL_CH2 | ADC_CTL_IE | ADC_CTL_END); // PE1

  ADCSequenceEnable (ADC0_BASE, 3);
  ADCSequenceEnable (ADC0_BASE, 2);
  ADCSequenceEnable (ADC0_BASE, 1);
  ADCIntClear (ADC0_BASE, 3);
  ADCIntClear (ADC0_BASE, 2);
  ADCIntClear (ADC0_BASE, 1);
}
void GPIO_Config ()
{
  SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOF);           // Enable port F
  GPIOPinTypeGPIOOutput (GPIO_PORTF_BASE, GPIO_PIN_3);    // green LED
  GPIOPinWrite (GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); // Turn on green LED

  SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOD); // Enable Port D PD0-PD3 are used to control the directions of the two motors

  GPIOPinTypeGPIOOutput (GPIO_PORTD_BASE, GPIO_PIN_0); // make PD0 and PD1 output pins WITH pull down resistors.
  GPIOPinTypeGPIOOutput (GPIO_PORTD_BASE, GPIO_PIN_1);
  GPIOPadConfigSet (GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPD);
  GPIOPadConfigSet (GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPD);

  GPIOPinTypeGPIOOutput (GPIO_PORTD_BASE, GPIO_PIN_2); // make PD2 and PD3 output pins WITH pull down resistors.
  GPIOPinTypeGPIOOutput (GPIO_PORTD_BASE, GPIO_PIN_3);
  GPIOPadConfigSet (GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPD);
  GPIOPadConfigSet (GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPD);
}
void MPU6050_Config () // initialize struct with register addresses
{
  MPU6050.address[0] = PWR_MGMT_1; // initialize struct
  MPU6050.address[1] = ACCEL_XOUT_H;
  MPU6050.address[2] = ACCEL_XOUT_L;
  MPU6050.address[3] = ACCEL_YOUT_H;
  MPU6050.address[4] = ACCEL_YOUT_L;
  MPU6050.address[5] = ACCEL_ZOUT_H;
  MPU6050.address[6] = ACCEL_ZOUT_L;
  MPU6050.address[7] = GYRO_XOUT_H;
  MPU6050.address[8] = GYRO_XOUT_L;
  MPU6050.address[9] = GYRO_YOUT_H;
  MPU6050.address[10] = GYRO_YOUT_L;
  MPU6050.address[11] = GYRO_ZOUT_H;
  MPU6050.address[12] = GYRO_ZOUT_L;

  I2CMasterSlaveAddrSet (I2C0_BASE, MPU6050_ADDRESS, false); // this code block sends a 0 byte to thw Power Managment register to turn on the MPU6050
  I2CMasterDataPut (I2C0_BASE, MPU6050.address[0]);
  //I2CMasterControl (I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
  I2CMasterControl (I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
  while (I2CMasterBusy (I2C0_BASE));
  I2CMasterDataPut (I2C0_BASE, 0);
  //I2CMasterControl (I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND_FINISH);
  while (I2CMasterBusy (I2C0_BASE));
}

void Timer_Config () // Timer 1 setup code           10 MS   used for calling blacklineInterrupt
{
  SysCtlPeripheralEnable (SYSCTL_PERIPH_TIMER1);    // enable Timer 1 periph clks
  TimerConfigure (TIMER1_BASE, TIMER_CFG_PERIODIC); // cfg Timer 1 mode - periodic
  uint32_t ui32Period = (SysCtlClockGet () / 100);  // period = 1/100th of a second AKA 10 MS
  TimerLoadSet (TIMER1_BASE, TIMER_A, ui32Period);  // set Timer 1 period
  TimerIntEnable (TIMER1_BASE, TIMER_TIMA_TIMEOUT); // enables Timer 1 to interrupt CPU
  TimerEnable (TIMER1_BASE, TIMER_A); // enable Timer 1
}

void UART_Config () // Configure UART1(Im using HC-05 Bluetooth)
{

  {
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOC); // Enable port C
    SysCtlPeripheralEnable (SYSCTL_PERIPH_UART1); // Enable Uart1
    GPIOPinConfigure (GPIO_PC4_U1RX);             // PC 4RX
    GPIOPinConfigure (GPIO_PC5_U1TX);             // PC5 TX
    GPIOPinTypeUART (GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    UARTClockSourceSet (UART1_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig (1, 9600, 16000000); // 9600 baud, 16MHz Clock
  }
}

void PWM_Config () // Enable PWM to contorl motor speeds.
{
  SysCtlPWMClockSet (SYSCTL_PWMDIV_1);
  SysCtlPeripheralEnable (SYSCTL_PERIPH_PWM0);
  SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOE);

  GPIOPinConfigure (GPIO_PE4_M0PWM4);                                                  // set pin for PWM(Alternate Function)
  GPIOPinTypePWM (GPIO_PORTE_BASE, GPIO_PIN_4);                                        // Set pin up for output mode.
  PWMGenConfigure (PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC); // count up/down mode(not sure what this means) and non syncronous.
  PWMGenPeriodSet (PWM0_BASE, PWM_GEN_2, 4000);                                        // set period of PWM 4000 clock ticks
  PWMGenEnable (PWM0_BASE, PWM_GEN_2);                                                 // allow clock to drive the timer for PWM

  GPIOPinConfigure (GPIO_PE5_M0PWM5);
  GPIOPinTypePWM (GPIO_PORTE_BASE, GPIO_PIN_5);
  PWMGenConfigure (PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
  PWMGenPeriodSet (PWM0_BASE, PWM_GEN_2, 4000);
  PWMGenEnable (PWM0_BASE, PWM_GEN_2);

  PWMOutputState (PWM0_BASE, PWM_OUT_4_BIT | PWM_OUT_5_BIT, true); // enable PWM outputs for PE4 and PE5
}
void leftMotorForward ()
{
  GPIOPinWrite (GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2); //
  GPIOPinWrite (GPIO_PORTD_BASE, GPIO_PIN_3, 0);
}
void leftMotorReverse ()
{
  GPIOPinWrite (GPIO_PORTD_BASE, GPIO_PIN_2, 0);
  GPIOPinWrite (GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3);
}
void rightMotorForward ()
{
  GPIOPinWrite (GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0);
  GPIOPinWrite (GPIO_PORTD_BASE, GPIO_PIN_1, 0);
}
void rightMotorReverse ()
{
  GPIOPinWrite (GPIO_PORTD_BASE, GPIO_PIN_0, 0);
  GPIOPinWrite (GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1); //
}
void leftMotorSetSpeed (int percentMotor) // this function takes an integer argument and sets the motor speed from 0-100% via pulse width modulation
{
  if (percentMotor < 35)
    percentMotor = 35;
  if (percentMotor > 100)
    percentMotor = 100;
  int width = percentMotor * 3999 / 100;
  PWMPulseWidthSet (PWM0_BASE, PWM_OUT_4, width);
}
void rightMotorSetSpeed (int percentMotor) // this function takes an integer argument and sets the motor speed from 0-100% via pulse width modulation
{
  if (percentMotor < 35)
    percentMotor = 35;
  if (percentMotor > 100)
    percentMotor = 100;
  int width = percentMotor * 3999 / 100;
  PWMPulseWidthSet (PWM0_BASE, PWM_OUT_5, width);
}
void I2C_Config () // Enable I2C protocol. PB2 is SCL and PB3 is SDA
{
  SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOB);
  SysCtlPeripheralEnable (SYSCTL_PERIPH_I2C0);
  GPIOPinConfigure (GPIO_PB2_I2C0SCL);
  GPIOPinConfigure (GPIO_PB3_I2C0SDA);
  GPIOPinTypeI2CSCL (GPIO_PORTB_BASE, GPIO_PIN_2);
  GPIOPinTypeI2C (GPIO_PORTB_BASE, GPIO_PIN_3);
  I2CMasterInitExpClk (I2C0_BASE, SysCtlClockGet (), false);
}

void getPIDConstants () // this function reads PID constants based on the PID potentiometers via ADC
{

  uint32_t ADCValue[1]; // This code block will read the voltage on PE3 and set kp_raw frm 0(0v) to 4096(3.3v)
  ADCProcessorTrigger (ADC0_BASE, 3);
  while (!ADCIntStatus (ADC0_BASE, 3, false))
    {
    }
  ADCIntClear (ADC0_BASE, 3);
  ADCSequenceDataGet (ADC0_BASE, 3, ADCValue);
  kp_raw = ADCValue[0]; // kp

  ADCProcessorTrigger (ADC0_BASE, 2);
  while (!ADCIntStatus (ADC0_BASE, 2, false))
    {
    }
  ADCIntClear (ADC0_BASE, 2);
  ADCSequenceDataGet (ADC0_BASE, 2, ADCValue);
  ki_raw = ADCValue[0]; // ki

  ADCProcessorTrigger (ADC0_BASE, 1);
  while (!ADCIntStatus (ADC0_BASE, 1, false))
    {
    }
  ADCIntClear (ADC0_BASE, 1);
  ADCSequenceDataGet (ADC0_BASE, 1, ADCValue);
  kd_raw = (ADCValue[0]); // kd

  // ADC IS 12 BIT AND RETURNS A INTEGER FROM 0-4096. We do not want our KP KI KD to be this high so we must scale it down
  kp = kp_raw * .02;
  ki = ki_raw * .000001;
  kd = kd_raw * .09;
}


void readData () // this function reads raw data from MPU6050 which will be used to calculate the robots tilt angle in a different function.
{
  AccelX = 0;
  AccelY = 0;
  AccelZ = 0;
  GyroX = 0;
  GyroY = 0;
  GyroZ = 0;

  int i = 0;
  for (i = 0; i < 4; i++) // take 4 samples so we can average it for better reading.
    {
      for (i = 1; i < 13; i++) // for each MPU6050 register(MPU6050.address) request data and store it in a variable(MPU6050.address)
        {
          I2CMasterSlaveAddrSet (I2C0_BASE, MPU6050_ADDRESS, false);
          I2CMasterDataPut (I2C0_BASE, MPU6050.address[i]);
          I2CMasterControl (I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
          while (I2CMasterBusy (I2C0_BASE))
            ;
          I2CMasterSlaveAddrSet (I2C0_BASE, MPU6050_ADDRESS, true);
          I2CMasterControl (I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
          while (I2CMasterBusy (I2C0_BASE))
            ;
          MPU6050.data[i] = (I2CMasterDataGet (I2C0_BASE));
        }

      AccelX += MPU6050.data[1] * 256 + MPU6050.data[2]; // Combine high byte and low byte (32 bit values)
      AccelY += MPU6050.data[3] * 256 + MPU6050.data[4] - GyroYOffset;
      ;
      AccelZ += MPU6050.data[5] * 256 + MPU6050.data[6];
      GyroX += MPU6050.data[7] * 256 + MPU6050.data[8];
      GyroY += MPU6050.data[9] * 256 + MPU6050.data[10];
      GyroZ += MPU6050.data[11] * 256 + MPU6050.data[12];
    }
  // divide by 4 for all values since we are taking the average of 4 samples.
  AccelX /= 4;
  AccelY /= 4;
  AccelZ /= 4;
  GyroX /= 4;
  GyroY /= 4;
  GyroZ /= 4;
}

void CalibrateGyroAndAccelerometer () // this is run once at the start to find the Gyro Offset and the Zero angle of the robot. Therefore when starting the robot make sure you are holding it upright at the angle you want it to balance at
{
  int i = 0;
  double sumgyro = 0;
  double sumaccel;
  for (i = 0; i < 100; i++) // for 100 iterations sum the raw Gyro Reading and the Angle calculated from Accelerometer
    {

      readData ();
      ////UARTprintf("%d \n",RawData[3]);
      sumgyro += GyroY;
      sumaccel += atan2 (AccelX, AccelZ) * 180.0 / M_PI;
      SysCtlDelay (SysCtlClockGet () / 100);
    }

  GyroYOffset = (sumgyro / 100.0); // divide by 100 to take average
  CalibratedZeroAngle = (sumaccel / 100.0);
  UARTprintf (" GyroYOffset: %d \n", GyroYOffset);
  UARTprintf ("Calibrated Zero Angle: %d \n", (int)CalibratedZeroAngle);
  // CalibratedZeroAngle=3;
  desiredAngle = CalibratedZeroAngle;
}



void GetAngle () // this function reads raw data that was taken from the MPU6050 and gives use the current tilt angle of the robot.
{

  double alpha = 0.98;                                                                                             // filter constant
  double dt = 0.01;                                                                                                // time between each Gyro reading.
  readData ();                                                                                                     // Read data from MPU6050
  double AngleFromAccelerometer = atan2 (AccelX, AccelZ) * 180.0 / M_PI;                                           // Calculate tilt angle from Accelerometer
  double ChangeInAngleFromGyroscope = -1.0 * GyroY * (250.0 / 32768.0) * dt;                                       // Calculate how many degrees the robot has tilted between the last time the function was called and now(dt) MPU6050 has min reading of -32768 and max reading of 32768(16 bit) and maximum/minimum of +-250 degrees/s
  FilteredAngle = alpha * (FilteredAngle + ChangeInAngleFromGyroscope) + (1.0 - alpha) * (AngleFromAccelerometer); // This is a complementary filter which combines gyroscope data and accelerometer data to provide a rather accurate tilt angle of the robot.
}

void PID()// This function is the main loop. It is called every 10 MS. It reads tilt angle and adjusts motors accordingly also reads remote control and adjusts accordingly
{
  iteration++;
  // UARTprintf("%d\n",(int)desiredAngle)
  char RemoteControl = UARTCharGetNonBlocking (UART1_BASE);
  if (RemoteControl == 'W')
    desiredAngle = CalibratedZeroAngle + 2; // makes robot go forward by increasing the angle at which the robot trys to balance

  if (RemoteControl == 'S')
    desiredAngle = CalibratedZeroAngle - 2; // makes robot go reverse by decreasing the angle at which the robot trys to balance

  if (RemoteControl == 'a') // the way the remote controller works is that you can only press one button at a time and when the button is released it will send 'a' which will return the robot to its normal balancing mode.
    {
      desiredAngle = CalibratedZeroAngle;
      turningleft = false;
      turningright = false;
    }
  if (RemoteControl == 'A')
    turningleft = true;

  if (RemoteControl == 'D')
    turningright = true;

  if (RemoteControl == 'u') // this increases the angle at which the robot trys to balance at(useful for fine tuning the angle to balance better)
    {
      CalibratedZeroAngle += .25;
      desiredAngle = CalibratedZeroAngle;
    }
  if (RemoteControl == 'd') // this decreases the angle at which the robot trys to balance at(useful for fine tuning the angle to balance better)
    {
      CalibratedZeroAngle -= .25;
      desiredAngle = CalibratedZeroAngle;
    }

  getPIDConstants (); // get PID contants from ADC's on the potentiometers

  GetAngle (); // Get angle calcualted from MPU6050

  double error = FilteredAngle*(-1.0) - desiredAngle; // we want angle to be 0 unless we want to drive forward/reverse. multiply by negative 1 because i installed MPU6050 backwards
  sumError += error;

  double P = kp * error;               // proportional
  double I = ki * sumError;            // integral(sum of all errors)
  double D = kd * (error - lasterror); // Derivative(difference between previous and current error)
  double PID = fabs (P) + I + D;

  lasterror = error;
  // UARTCharPut(UART1_BASE,(int)FilteredAngle);
  // UARTprintf("Current Angle: %d    DesiredAngle: %d \n",(int)FilteredAngle,(int)desiredAngle);
  // UARTprintf("Robot Angle: %d.%d  KP:0.005 *  %d  KI:0.0001 * %d  KD: 0.005 * %d   PID:%d\r",(int)FilteredAngle, abs((int)(10*(FilteredAngle-(int)FilteredAngle))),(int)kp,(int)ki,(int)kd,(int)PID);
  if ((turningleft) && (iteration % 5 == 0)) // This turns the robot by pulsing the motors in oppisite directions at 100% speed every fifth PID iteration(only every fifth so the robot stays balancing) this is probably not the most elegeant way of turning the robot but this is all I could think of.
    {
      rightMotorForward ();
      leftMotorReverse ();
      leftMotorSetSpeed (100);
      rightMotorSetSpeed (100);
    }
  else if ((turningright) && (iteration % 5 == 0)) // This turns the robot by pulsing the motors in oppisite directions at 100% speed every fifth PID iteration(only every fifth so the robot stays balancing) this is probably not the most elegeant way of turning the robot but this is all I could think of.
    {
      leftMotorForward ();
      rightMotorReverse ();
      leftMotorSetSpeed (100);
      rightMotorSetSpeed (100);
    }

  else if (error > desiredAngle) // If robot tilting forwards then drive robot forward
    {
      rightMotorForward ();
      leftMotorForward ();
      leftMotorSetSpeed (PID);
      rightMotorSetSpeed (PID);
    }
  else // If robot tilting forwards then drive robot forward
    {
      rightMotorReverse ();
      leftMotorReverse ();
      leftMotorSetSpeed (PID);
      rightMotorSetSpeed (PID);
    }
}

int main (void)
{
  SysCtlClockSet (SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
  ADC_Config ();
  GPIO_Config ();
  PWM_Config ();
  UART_Config ();
  I2C_Config ();
  MPU6050_Config ();
  CalibrateGyroAndAccelerometer ();
  Timer_Config ();
  BIOS_start ();
}
