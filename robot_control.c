#include "driverlib.h"
#include "header.h"
#include <stdio.h>

//variables for PID
#define Kd 0.2
#define Ki 300
#define Kp 30
#define PWMDutyCycle 1000
#define PIDfreq 60

volatile float cumerror1 = 0;
volatile float preverror1 = 0;
volatile float cumerror2 = 0;
volatile float preverror2 = 0;
volatile float cumerror3 = 0;
volatile float preverror3 = 0;

//variables for direction P control and desired values
#define Khead 0.1

int desheading = 0;
int actheading = 0;
int desspeed = 0;

//variables for encoder
volatile float speed1 = 0;
volatile unsigned int prevtime1 = 0;
volatile int state1 = 0;

volatile float speed2 = 0;
volatile unsigned int prevtime2 = 0;
volatile int state2 = 0;

volatile float speed3 = 0;
volatile unsigned int prevtime3 = 0;
volatile int state3 = 0;

// variables for velocity update
volatile int updatelowlevel = 0;
int desvel1 = 0;
int desvel2 = 0;
int desvel3 = 0;

//variables for heading update
volatile int updatehead = 0;

volatile int repS1 = 0;
volatile int repS2 = 0;

volatile float data[300] = {};
volatile int idx = 0;
int idx1 = 0;

//configure control update timer
const Timer_A_UpModeConfig configuration =
    {   TIMER_A_CLOCKSOURCE_SMCLK, //use submaster clock
        TIMER_A_CLOCKSOURCE_DIVIDER_64, //divide by 64
        3E+6/(64*PIDfreq),// Period of Timer A (this value placed in TAxCCR0)
        TIMER_A_TAIE_INTERRUPT_DISABLE,// Enable Timer A rollover interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,// Disable Capture Compare interrupt
        TIMER_A_DO_CLEAR// Clear counter upon initialization
    };

void setup(void){

    MAP_FPU_enableModule();
    MAP_WDT_A_holdTimer();      // stop watchdog timer

    //Setup pins
    MAP_GPIO_setAsInputPinWithPullUpResistor(SWITCH1); // set up Switch1
    MAP_GPIO_setAsInputPinWithPullUpResistor(SWITCH2); // set up Switch2
    MAP_GPIO_setAsInputPin(ENCODER1);
    MAP_GPIO_setAsInputPin(ENCODER2);
    MAP_GPIO_setAsInputPin(ENCODER3);
    MAP_GPIO_setAsOutputPin(LED1); //set up LED1
    MAP_GPIO_setAsOutputPin(CHECK);

    //initialize LEDs off
    MAP_GPIO_setOutputLowOnPin(LED1);
    MAP_GPIO_setOutputLowOnPin(CHECK);

    //setup clock
    unsigned int dcoFrequency = 3E+6;  // set clock frequency to 3MHz
    MAP_CS_setDCOFrequency (dcoFrequency);
    MAP_CS_initClockSignal(CS_SMCLK,CS_DCOCLK_SELECT,CS_CLOCK_DIVIDER_1);

    //setup interrupts
    MAP_Interrupt_disableMaster();
    MAP_GPIO_interruptEdgeSelect (SWITCH1, GPIO_HIGH_TO_LOW_TRANSITION);
    MAP_GPIO_interruptEdgeSelect (SWITCH2, GPIO_HIGH_TO_LOW_TRANSITION);
    MAP_GPIO_interruptEdgeSelect (ENCODER1, GPIO_LOW_TO_HIGH_TRANSITION);
    MAP_GPIO_interruptEdgeSelect (ENCODER2, GPIO_LOW_TO_HIGH_TRANSITION);
    MAP_GPIO_interruptEdgeSelect (ENCODER3, GPIO_LOW_TO_HIGH_TRANSITION);
    MAP_GPIO_clearInterruptFlag (SWITCH1);
    MAP_GPIO_clearInterruptFlag (SWITCH2);
    MAP_GPIO_clearInterruptFlag (ENCODER1);
    MAP_GPIO_clearInterruptFlag (ENCODER2);
    MAP_GPIO_clearInterruptFlag (ENCODER3);
    MAP_GPIO_enableInterrupt (SWITCH1);
    MAP_GPIO_enableInterrupt (SWITCH2);
    MAP_GPIO_enableInterrupt (ENCODER1);
    MAP_GPIO_enableInterrupt (ENCODER2);
    MAP_GPIO_enableInterrupt (ENCODER3);
    Interrupt_enableInterrupt (INT_PORT1);
    Interrupt_enableInterrupt (INT_PORT3);
    Interrupt_setPriority(INT_PORT3, 0x00);

    // setup PWM
    //motor 1 pins 6.6 and 6.7
    P6SEL0 |= 0x40; // Set bit 6 of P6SEL0 to enable TA2.3 functionality on P6.6
    P6DIR |= 0x40;// Set pin 6.6 as an output pin
    P6SEL1 &= ~0x40;
    P6SEL0 |= 0x80; // Set bit 6 of P5SEL0 to enable TA2.4 functionality on P6.7
    P6DIR |= 0x80;// Set pin 6.7 as an output pin
    P6SEL1 &= ~0x80;
        //timer TA2 capture compare
    TA2CCR0 = PWMDutyCycle ;    // Set Duty cycle
    TA2CCR3 = 0 ;    // Set output mode to Reset/Set
    TA2CCR4 = 0 ;    // Set output mode to Reset/Set
    TA2CCTL3 = OUTMOD_7 ;
    TA2CCTL4 = OUTMOD_7 ;
    //motor 2 pins 5.6 and 5.7
    P5SEL0 |= 0x40; // Set bit 6 of P6SEL0 to enable TA2.1 functionality on P6.6
    P5DIR |= 0x40;// Set pin 6.6 as an output pin
    P5SEL1 &= ~0x40;
    P5SEL0 |= 0x80; // Set bit 6 of P5SEL0 to enable TA2.2 functionality on P6.7
    P5DIR |= 0x80;// Set pin 6.7 as an output pin
    P5SEL1 &= ~0x80;
        //timer TA2 capture compare duty cycle set above
    TA2CCR1 = 0 ;    // Set output mode to Reset/Set
    TA2CCR2 = 0 ;    // Set output mode to Reset/Set
    TA2CCTL1 = OUTMOD_7 ;
    TA2CCTL2 = OUTMOD_7 ;
    //motor 3 pins 2.6 and 2.7
    P2SEL0 |= 0x40; // Set bit 6 of P6SEL0 to enable TA2.3 functionality on P6.6
    P2DIR |= 0x40;// Set pin 6.6 as an output pin
    P2SEL1 &= ~0x40;
    P2SEL0 |= 0x80; // Set bit 6 of P5SEL0 to enable TA2.4 functionality on P6.7
    P2DIR |= 0x80;// Set pin 6.7 as an output pin
    P2SEL1 &= ~0x80;
            //timer TA0 capture compare
    TA0CCR0 = PWMDutyCycle ;    // Set Duty cycle
    TA0CCR3 = 0 ;    // Set output mode to Reset/Set
    TA0CCR4 = 0 ;    // Set output mode to Reset/Set
    TA0CCTL3 = OUTMOD_7 ;
    TA0CCTL4 = OUTMOD_7 ;


    // Initialize Timer A2 and A0
    TA2CTL = TASSEL__SMCLK | MC__UP | TACLR ;// Tie Timer A2 to SMCLK, use Up mode, and clear TA2R
    TA0CTL = TASSEL__SMCLK | MC__UP | TACLR ;// Tie Timer A0 to SMCLK, use Up mode, and clear TA0R

    //setup timer for speed update
    MAP_SysTick_enableModule();
    MAP_SysTick_setPeriod(3000000);
    MAP_SysTick_disableInterrupt ();

    //setup Timer A1 for control update
    MAP_Timer_A_configureUpMode(TIMER_A1_BASE, &configuration);  // Configure Timer A1 using above struct
    MAP_Interrupt_enableInterrupt(INT_TA1_0);// Enable Timer A1 interrupt
    MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);  // Start Timer A1
    MAP_Interrupt_setPriority(INT_TA1_0, 0x01);

    // Enabling interrupts
    Interrupt_enableMaster();
    }

void updatecontrol(void){
    //PID motor 1
    float error1 = abs(desvel1) - speed1;
    cumerror1 += (error1 + preverror1)/(2*PIDfreq);
    float PID1 = Kp*error1 + Ki*cumerror1 + Kd*(error1-preverror1)*PIDfreq;
    if (PID1 < 0) {PID1 = 0;}
    if (PID1 > PWMDutyCycle){
        PID1 = PWMDutyCycle;
        cumerror1 -= (error1 + preverror1)/(2*PIDfreq); //clamping anti-windup
    }
    //PID motor 2
    float error2 = abs(desvel2) - speed2;
    cumerror2 += (error2 + preverror2)/(2*PIDfreq);
    float PID2 = Kp*error2 + Ki*cumerror2 + Kd*(error2-preverror2)*PIDfreq;
    if (PID2 < 0) {PID2 = 0;}
    if (PID2 > PWMDutyCycle){
        PID2 = PWMDutyCycle;
        cumerror2 -= (error2 + preverror2)/(2*PIDfreq); //clamping anti-windup
    }
    //PID motor 3
    float error3 = abs(desvel3) - speed3;
    cumerror3 += (error3 + preverror3)/(2*PIDfreq);
    float PID3 = Kp*error3 + Ki*cumerror3 + Kd*(error3-preverror3)*PIDfreq;
    if (PID3 < 0) {PID3 = 0;}
    if (PID3 > PWMDutyCycle){
        PID3 = PWMDutyCycle;
        cumerror3 -= (error3 + preverror3)/(2*PIDfreq); //clamping anti-windup
    }

    //Avoid quick switches of motor direction
    if (sgn(speed1) ~= sgn(PID1) & speed1 > 10) {PID1 = 0;}
    if (sgn(speed2) ~= sgn(PID2) & speed2 > 10) {PID2 = 0;}
    if (sgn(speed3) ~= sgn(PID3) & speed3 > 10) {PID3 = 0;}

    //Write outputs to motors
    if (desvel1 > 0){
        TA2CCR3 = PID1;
        TA2CCR4 = 0;
    }
    else {
        TA2CCR4 = PID1;
        TA2CCR3 = 0;
    }
    if (desvel2 > 0){
        TA2CCR1 = PID2;
        TA2CCR2 = 0;
    }
    else {
        TA2CCR2 = PID2;
        TA2CCR1 = 0;
    }
    if (desvel3 > 0){
        TA0CCR3 = PID3;
        TA0CCR4 = 0;
    }
    else {
        TA0CCR4 = PID3;
        TA0CCR3 = 0;
    }
    updatelowlevel = 0;
}

void updateheading(void){
    // P control for heading
    headerror = yaw_desired - yaw_actual;
    headvel = Khead*headerror;

    // apply dynamics equations
    factor1 = 10 //scale to match rpm
    desvel1 = factor*roll - headvel;
    desvel2 = -(sqrt(3)/2)*factor*pitch - factor*roll/2 - headvel;
    desvel3 = (sqrt(3)/2)*factor*pitch - factor*roll/2 - headvel;

    // handle saturation
    if (desvel1 > 140) {desvel1 = 140;}
    else if (desvel1 < 40) {desvel1 = 0;}
    if (desvel2 > 140) {desvel2 = 140;}
    else if (desvel2 < 40) {desvel2 = 0;}
    if (desvel3 > 140) {desvel3 = 140;}
    else if (desvel3 < 40) {desvel3 = 0;}
}

void main(void){
    setup();

    while(1){
       if (updatehead == 1){
           updateheading();
       }

       if (updatelowlevel == 1){
            updatecontrol();
            idx1 = idx1 + 1;
       }
            if (repS2 == 1){
                int i;
                for (i = 0; i <= 300; i++ ){
                    printf("%0.1f \n",data[i]);
                }
                repS2 = 0;
            }
    }
}

void PORT1_IRQHandler (void){
        if(P1IFG&BIT1){
            repS1 = 1;
            P1IFG &= ~ BIT1;
        }
        else if(P1IFG&BIT4){
            repS2 = 1;
            P1IFG &= ~ BIT4;
        }
}

void PORT3_IRQHandler (void){
    MAP_GPIO_toggleOutputOnPin(CHECK);
        //encoder for motor 1
        if(P3IFG&BIT5){
            unsigned int time1 = MAP_SysTick_getValue();
            if (state1 == 0){
                MAP_GPIO_interruptEdgeSelect (ENCODER1, GPIO_HIGH_TO_LOW_TRANSITION);
                state1 = 1;
            }
            else {
                MAP_GPIO_interruptEdgeSelect (ENCODER1, GPIO_LOW_TO_HIGH_TRANSITION);
                state1 = 0;
            }
            P3IFG &= ~ BIT5;
            long int timediff = prevtime1 - time1;
            if (timediff < 0){
                timediff = timediff + 3000000;
            }
            speed1 = (3E+6)/(2*timediff); //revolutions/second
            prevtime1 = time1;
        }
        //encoder for motor 2
        else if(P3IFG&BIT6){
            unsigned int time2 = MAP_SysTick_getValue();
            if (state2 == 0){
                MAP_GPIO_interruptEdgeSelect (ENCODER2, GPIO_HIGH_TO_LOW_TRANSITION);
                state2 = 1;
            }
            else {
                MAP_GPIO_interruptEdgeSelect (ENCODER2, GPIO_LOW_TO_HIGH_TRANSITION);
                state2 = 0;
            }
            P3IFG &= ~ BIT6;
            long int timediff = prevtime2 - time2;
            if (timediff < 0){
                timediff = timediff + 3000000;
            }
            speed2 = (3E+6)/(2*timediff); //revolutions/second
            prevtime2 = time2;
        }
        //encoder for motor 3
        else if(P3IFG&BIT7){
            unsigned int time3 = MAP_SysTick_getValue();
            if (state3 == 0){
                MAP_GPIO_interruptEdgeSelect (ENCODER3, GPIO_HIGH_TO_LOW_TRANSITION);
                state3 = 1;
            }
            else {
                MAP_GPIO_interruptEdgeSelect (ENCODER3, GPIO_LOW_TO_HIGH_TRANSITION);
                state3 = 0;
            }
            P3IFG &= ~ BIT7;
            long int timediff = prevtime3 - time3;
            if (timediff < 0){
                timediff = timediff + 3000000;
            }
            speed3 = (3E+6)/(2*timediff); //revolutions/second
            prevtime3 = time3;
        }

        if (idx <= 300){ // for data output
           data[idx]= speed1;
           idx += 1;
        }
}

void TA1_0_IRQHandler (void){
    MAP_GPIO_toggleOutputOnPin(LED1); //flash LED
    updatelowlevel = 1;
    MAP_Timer_A_clearInterruptFlag (TIMER_A1_BASE);
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
}

