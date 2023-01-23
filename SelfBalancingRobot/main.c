
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/GPIO.h>

#include "Board.h"

//*****************************************************************************
// Zachary Hillary
// 12/17/2022
// Self Balancing Robot Project
//
//PB2:SCL
//PB3:SDA
// PC4: RX (TX ON BLUETOOTH)
//PC3 : TX (RX ON BLUETOOTH)
//PE3: KP
//PE2: KI
//PE1: KD
//PE4:Left Motor Speed(PWM)
//PE5:Right Motor Speed(PWM)
//PD0 & PD1 : Right Motor Direction
//PD2 & PD3 : Left Motor Direction
//
//
//*****************************************************************************
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h> //adds new integer data types
#include <stdbool.h> //adds bool data type
#include "inc/hw_memmap.h" //Map of registers for the TI EK123gxl board
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/i2c.h"
#include "inc/hw_i2c.h"
#include <math.h>
#include "driverlib/adc.h"
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include "driverlib/timer.h"
#include <ti/sysbios/knl/Task.h>
#define MPU6050_ADDRESS  0x68  // device address for MPU_6050

#define PWR_MGMT_1 0x6B     // register addresses for MPU_6050 I2C
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

typedef struct MPU6050_REGISTERS{
    int address[13];
   unsigned int data[13];
}MPU6050_REGISTERS;

MPU6050_REGISTERS MPU6050;//create new instance of a struct

void MPU6050_Config() //initialize struct with register addresses
{
MPU6050.address[0] =PWR_MGMT_1; //initialize struct
MPU6050.address[1] =ACCEL_XOUT_H;
MPU6050.address[2] =ACCEL_XOUT_L;
MPU6050.address[3] =ACCEL_YOUT_H;
MPU6050.address[4] =ACCEL_YOUT_L;
MPU6050.address[5] =ACCEL_ZOUT_H;
MPU6050.address[6] =ACCEL_ZOUT_L;
MPU6050.address[7] =GYRO_XOUT_H;
MPU6050.address[8] =GYRO_XOUT_L;
MPU6050.address[9] =GYRO_YOUT_H;
MPU6050.address[10] =GYRO_YOUT_L;
MPU6050.address[11] =GYRO_ZOUT_H;
MPU6050.address[12] =GYRO_ZOUT_L;
        I2CMasterSlaveAddrSet( I2C0_BASE, MPU6050_ADDRESS , false );
        I2CMasterDataPut( I2C0_BASE, MPU6050.address[0] );
        I2CMasterControl( I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START );
        while( I2CMasterBusy( I2C0_BASE ) );
        I2CMasterDataPut( I2C0_BASE, 0);
        I2CMasterControl( I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH );
        while( I2CMasterBusy( I2C0_BASE ) );
}

void Timer_Config()                              // Timer 1 setup code           10 MS   used for calling blacklineInterrupt
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);    // enable Timer 1 periph clks
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC); // cfg Timer 1 mode - periodic
    uint32_t ui32Period = (SysCtlClockGet() / 1000);  // period = 1/1000th of a second AKA 1 MS
    TimerLoadSet(TIMER1_BASE, TIMER_A, ui32Period);  // set Timer 1 period

    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT); // enables Timer 1 to interrupt CPU

    TimerEnable(TIMER1_BASE, TIMER_A);                      // enable Timer 1
}


// Configures ADC usage for PE3 PE2 PE1
void ADC_Config()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1);

    ADCSequenceDisable(ADC0_BASE, 1);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);  //PE3


    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);  //PE2

    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH2 | ADC_CTL_IE | ADC_CTL_END);  //PE1

    ADCSequenceEnable(ADC0_BASE, 3);
    ADCSequenceEnable(ADC0_BASE, 2);
    ADCSequenceEnable(ADC0_BASE, 1);
    ADCIntClear(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 2);
    ADCIntClear(ADC0_BASE, 1);
}

char RemoteControl;
void UARTHandler(void)
{
    uint32_t ui32Status;

    // Get the interrrupt status.
    ui32Status = UARTIntStatus(UART1_BASE, true);

    // Clear the asserted interrupts.
    UARTIntClear(UART1_BASE, ui32Status);

    // Loop while there are characters in the receive FIFO.
    while(UARTCharsAvail(UART1_BASE))
    {
        // Read the next character from the UART and print it.
        RemoteControl = UARTCharGetNonBlocking(UART1_BASE);

    }
}




void UART_Config()
{
   // SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
   // GPIOPinConfigure(GPIO_PA0_U0RX);
    //GPIOPinConfigure(GPIO_PA1_U0TX);
   // SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    //UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
   // GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
   // UARTStdioConfig(0, 115200, 16000000);

    {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
        GPIOPinConfigure(GPIO_PC4_U1RX);  //PC4RX PC5TX
        GPIOPinConfigure(GPIO_PC5_U1TX);
        GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
        UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);
        UARTStdioConfig(1, 9600, 16000000);

    }
}
void GPIO_Config()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);//red LED
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1); //green LED
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPadConfigSet(GPIO_PORTD_BASE,GPIO_PIN_0,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD_WPD); //Use pull down because somehow PD0 and PD1 are connected to eachother.
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTD_BASE,GPIO_PIN_1,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD_WPD);
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2);
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_3);
    GPIOPadConfigSet(GPIO_PORTD_BASE,GPIO_PIN_2,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD_WPD);
    GPIOPadConfigSet(GPIO_PORTD_BASE,GPIO_PIN_3,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD_WPD);

}
void PWM_Config(){
       SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
       SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
       GPIOPinConfigure(GPIO_PE4_M0PWM4);
       GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);
       PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
       PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 4000);
       PWMGenEnable(PWM0_BASE, PWM_GEN_2);
       GPIOPinConfigure(GPIO_PE5_M0PWM5);
       GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_5);
       PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
       PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 4000);
       PWMGenEnable(PWM0_BASE, PWM_GEN_2);
       PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT | PWM_OUT_5_BIT, true);
}
void leftMotorForward(){
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3);
}
void leftMotorReverse(){
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2);//
       GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0);
}
void rightMotorForward(){
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0);
          GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1);//
}
void rightMotorReverse(){

       GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0);
           GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0);
}
void leftMotorSetSpeed(int percentMotor){
    if (percentMotor<35)percentMotor=35;
    if (percentMotor>100)percentMotor=100;
    int width=percentMotor*3999/100;
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4,width);
}
void rightMotorSetSpeed(int percentMotor){
   if (percentMotor<35)percentMotor=35;
    if (percentMotor>100)percentMotor=100;
    int width=percentMotor*3999/100;
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5,width);
}
void I2C_Config(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);


}
double desiredAngle;
double kp,ki,kd;
int kp_raw,ki_raw,kd_raw;
double CalibratedZeroAngle;
void getPIDConstants(){
    uint32_t ADCValue[1];
    ADCProcessorTrigger(ADC0_BASE, 3);
    while (!ADCIntStatus(ADC0_BASE, 3, false)) { }
    ADCIntClear(ADC0_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, ADCValue);
    kp_raw = ADCValue[0];  //kp

    ADCProcessorTrigger(ADC0_BASE, 2);
    while (!ADCIntStatus(ADC0_BASE, 2, false)) { }
    ADCIntClear(ADC0_BASE, 2);
    ADCSequenceDataGet(ADC0_BASE, 2, ADCValue);
    ki_raw = ADCValue[0];//ki

     ADCProcessorTrigger(ADC0_BASE, 1);
     while (!ADCIntStatus(ADC0_BASE, 1, false)) { }
     ADCIntClear(ADC0_BASE, 1);
     ADCSequenceDataGet(ADC0_BASE, 1, ADCValue);
     kd_raw = (ADCValue[0]);  //kd

     kp= kp_raw*.02;
    ki=ki_raw*.00001;
     //CalibratedZeroAngle=(ki_raw/4096.0)*14.0-5.0;
     kd=kd_raw*.01;
     if(kp_raw<100)kp=0;
      if(ki_raw<100)ki=0;
      if(kd_raw<100)kd=0;
}
int GyroYOffset=0;
int16_t AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ;
void readData(){

int i=0;
for(i=1;i<13;i++){
     I2CMasterSlaveAddrSet(I2C0_BASE, MPU6050_ADDRESS, false);
     I2CMasterDataPut(I2C0_BASE, MPU6050.address[i]);
     I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
     while(I2CMasterBusy(I2C0_BASE));
     I2CMasterSlaveAddrSet(I2C0_BASE, MPU6050_ADDRESS, true);
     I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
     while(I2CMasterBusy(I2C0_BASE));
     MPU6050.data[i] =( I2CMasterDataGet(I2C0_BASE));
}

AccelX = MPU6050.data[1]*256 + MPU6050.data[2];
AccelY = MPU6050.data[3]*256 + MPU6050.data[4];
AccelZ = MPU6050.data[5]*256 + MPU6050.data[6];
GyroX = MPU6050.data[7]*256 + MPU6050.data[8];
GyroY = MPU6050.data[9]*256 + MPU6050.data[10]-GyroYOffset;
GyroZ = MPU6050.data[11]*256 + MPU6050.data[12];



}



void CalibrateGyroAndAccelerometer(){
    int i=0;
    double sumgyro=0;
    double sumaccel;
    for (i=0;i<100;i++){

        readData();
if(i%3==0)GPIO_toggle(2);//blink red
        sumgyro+=GyroY;
        //sumaccel+= -1.0*atan2(AccelX, AccelZ)*180.0/M_PI;
        //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);//Turn on red LED
        SysCtlDelay(SysCtlClockGet()/100);
        //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);//Turn off red LED
    }
    GyroYOffset=(sumgyro/100.0);
    //CalibratedZeroAngle = (sumaccel/100.0);
    UARTprintf(" GyroYOffset: %d \n",GyroYOffset);
    UARTprintf("Calibrated Zero Angle: %d \n", (int)CalibratedZeroAngle);
    CalibratedZeroAngle=-3;
    desiredAngle=CalibratedZeroAngle;
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);//Turn on green LED
}


double lasterror = 0;
double sumError=0;
double FilteredAngle=0;

void GetAngle(){


    double alpha=0.98; //filter constant
       double dt = 0.01;//time between each Gyro reading.
       readData();
       double AngleFromAccelerometer = atan2(AccelX, AccelZ)*180.0/M_PI;
       double   ChangeInAngleFromGyroscope = -1.0*GyroY*(250.0/32768.0)*dt;
       FilteredAngle = alpha*(FilteredAngle + ChangeInAngleFromGyroscope)+(1.0-alpha)*(AngleFromAccelerometer);
}
int forward=false;
int reverse=false;
int turningleft=false;
int turningright=false;
int iteration;
void PID(){
iteration++;
//UARTprintf("%d\n",(int)desiredAngle)
   char RemoteControl =  UARTCharGetNonBlocking(UART1_BASE);
  if (RemoteControl == 'W')desiredAngle=CalibratedZeroAngle-2;//forward=true;//forward



   if (RemoteControl == 'S')desiredAngle=CalibratedZeroAngle+2;//reverse=true;//reverse

   if (RemoteControl == 'a'){
       desiredAngle=CalibratedZeroAngle;//balance
       turningright=false;
       turningleft=false;
       //forward=false;
       //reverse=false;
   }



   if(RemoteControl=='A')turningleft=true;//left

   if (RemoteControl=='D')turningright=true;//right
   if (RemoteControl == 'u') // this increases the angle at which the robot trys to balance at(useful for fine tuning the angle to balance better)
       {
         CalibratedZeroAngle += .5;
         desiredAngle = CalibratedZeroAngle;
       }
     if (RemoteControl == 'd') // this decreases the angle at which the robot trys to balance at(useful for fine tuning the angle to balance better)
       {
         CalibratedZeroAngle -= .5;
         desiredAngle = CalibratedZeroAngle;
       }
          getPIDConstants();

       GetAngle();

    double error = -1*FilteredAngle-desiredAngle;
    sumError+=error;

    double P = kp*error;                      //proportional
    double I =ki*sumError;                   //integral(sum of all errors)
    double D = kd*(error-lasterror);         //Derivative(difference between previous and current error)
    double PID = P+I+D;

    lasterror=error;
//UARTCharPut(UART1_BASE,(int)FilteredAngle);

    //UARTprintf("Robot Angle: %d.%d  KP:0.005 *  %d  KI:0.0001 * %d  KD: 0.005 * %d   PID:%d\r",(int)FilteredAngle, abs((int)(10*(FilteredAngle-(int)FilteredAngle))),(int)kp,(int)ki,(int)kd,(int)PID);
if((turningright)&&(iteration%3==0)){
           rightMotorForward();
           leftMotorReverse();
           leftMotorSetSpeed(100);
           rightMotorSetSpeed(100);
       }
       else if((turningleft)&&(iteration%3==0)){
           leftMotorForward();
           rightMotorReverse();
           leftMotorSetSpeed(100);
            rightMotorSetSpeed(100);
       }
       /*else if((forward)&&(iteration%2==0)){
                  leftMotorForward();
                  rightMotorForward();
                  leftMotorSetSpeed(100);
                   rightMotorSetSpeed(100);
              }
       else if((reverse)&&(iteration%2==0)){
                  leftMotorReverse();
                  rightMotorReverse();
                  leftMotorSetSpeed(100);
                   rightMotorSetSpeed(100);
              }*/
       else if (PID>0)
    {
        rightMotorReverse();
        leftMotorReverse();
        leftMotorSetSpeed(fabs(PID));
        rightMotorSetSpeed(fabs(PID));
    }
    else{
        rightMotorForward();
       leftMotorForward();
      leftMotorSetSpeed(fabs(PID));
        rightMotorSetSpeed(fabs(PID));
    }


}

int
main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

  ADC_Config();
  GPIO_Config();
  PWM_Config();

  UART_Config();
  //UARTprintf("hi");
  I2C_Config();
  MPU6050_Config();
  CalibrateGyroAndAccelerometer();

  Timer_Config();


 BIOS_start();

}

