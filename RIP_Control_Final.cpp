////////////////////////////////////////////////////////////////////////////////
//                  System Identification for DC Motors                       //
//                    A is for left B is for right                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/*
  - Modify the Hardware setting about PWM. Setting register that related to PWM output
    directly, instead of using Mbed's API about PWM.
  - It may provide your motor better performance.
*/

#include "mbed.h"
#include <math.h>
#define pi 3.1415f  


#define INPUT_VOLTAGE       12.0f
#define VOLT_LIMIT          12.0f
#define PWM_FREQUENCY       10.0f       // (unit : kHz) you may need to change. Recommand using 10.0 (10 kHz)
#define HALL_RESOLUTION     2000.0f     // Encoder number per one circle HALL SENSOR
#define PWM_STOP            0.5f

#define J1 0.001569f
#define m2 0.022f
#define l1 0.16f
#define C2 0.08f
#define J2 0.0001785f
#define Kt 0.01826f
#define Rm 2.5604f
#define g  9.8f
////////////////////////
float Mr11=0.0f;
float Mr12=0.0f;
float Mr21=0.0f;
float Mr22=0.0f;
float Gr2=0.0f;
float Mr11bar=0.0f;
float Gr1bar=0.0f;



Serial pc(USBTX,USBRX);
InterruptIn mybutton(USER_BUTTON);
Ticker main_function;               // interrupt function
PwmOut pwm1A(D7);
PwmOut pwm1B(D8);
PwmOut pwm2A(D11);
PwmOut pwm2B(A3);
DigitalOut led1(LED1);

float dt = 0.001f; // unit:second
float theta1 = 0;
float u = 0;  // voltage to motor (control value)
bool  button_state = false;
float dutycycle = PWM_STOP;
float you_need_to_do = 0.5;


float encoder1=0.0f;
float encoder2=0.0f;
float position1 = 0.0f;
float position2 = 0.0f;
float last_encoder1=0.0f;
float last_encoder2=0.0f;
float last_position1 = 0.0f;
float last_position2 = 0.0f;
float last_velocity1=0.0f;
float last_velocity2=0.0f;
float velocity1=0.0f;
float velocity2=0.0f;
float Ks=20.0f;
float position2_2 = 0.0f;
float K1 = you_need_to_do;
float K2 = you_need_to_do;
float K3 = you_need_to_do;
float K4 = you_need_to_do;
float sample_K[4]={-9.8009,-114.4438,-9.2241,-15.4768};
float our_K[4] = {-6.3698,-73.8980,-3.6537,-9.9319};
float digital_K[4] = {-3.3937,-92.4320,-4.0798,-12.8195};
void step_command();
void position_control();
void ReadVelocity();
void ReadPosition();
void motor_drive(float voltA, float voltB);
void InitMotor(float pwm_frequency);
void InitEncoder(void);
void UpdateTheta();
void RIP_control();

uint16_t CCR_value(float duty);

int main()
{
    pc.baud(115200);            //default baud rate
    InitEncoder();              //don't change it
    InitMotor(PWM_FREQUENCY);   // Set pwm period. Don't change the contents in this function unless you know what you do
    mybutton.fall(&step_command);   // if you push blue botton, step_command will work
    main_function.attach_us(&RIP_control, dt*1000000); //dt*1000000 changing unit to second.

    while(1) {
        //only printf one
        //pc.printf("%.5f  %.5f\r\n",velocity1,velocity2);
        pc.printf("%.5f\r\n",position2_2);
        //pc.printf("%.5f\r\n",u);
        //pc.printf("%.5f  %.5f   %.5f\r\n",u,position1,position2);
//        pc.printf("%.5f,%.5f\r\n",encoder1,last_encoder1);     // about 400us
        //pc.printf("%d\r\n",TIM3->CNT);
        wait_ms(10);
    }
}



void step_command()
{
    led1 = !led1;
    button_state = !button_state;
    //button_state = true;
    
    // init x state
    ///////////////////////////////////////////////////////
    position1 = 0;
    position2 = -pi;
    velocity1 = 0;
    velocity2 = 0;
    ///////////////////////////////////////////////////////
}

void RIP_control()
{
    UpdateTheta();
    if(button_state) 
    {
        // decide position2 to change stage
        while(position2<(-2.0f*pi)) {
            position2 += 2.0f*pi;
        }
        while(position2>0) {
            position2 -= 2.0f*pi;
        }
        if(position2<=(-30.0f*pi/180.0f)&&position2>=(-330.0f*pi/180.0f))
        {
            //you need define angle
            ///////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////
            float ur1;
            if(velocity2*cos(position2)>0.0f) 
            {
                ur1 = -Ks;
            } else if(velocity2*cos(position2)<0.0f) 
            {
                ur1 = Ks;
            } else 
            {
                ur1 = 0.0f;
            }

            Mr11=J1+m2*l1*l1+m2*C2*C2*sin(position2)*sin(position2);
            Mr12=m2*l1*C2*cos(position2);
            Mr21=Mr12;
            Mr22=J2+m2*C2*C2;
            Gr2=-m2*C2*g*sin(position2);
            Mr11bar=Mr11-Mr12*Mr21/Mr22;
            Gr1bar=-Mr12*Gr2/Mr22;
            u=Rm*(Mr11bar*ur1+Kt*Kt*velocity1/Rm+Gr1bar)/Kt;    
            motor_drive(u,0);                                   // swing Inverted Pendulum

        } else 
        {

            ///////////////////////////////////////////////////////
            while(abs(position1)>pi)
            {
                 if(position1>0)
                {
                    position1 = position1 - 2*pi;
                }else
                {
                    position1 = 2*pi + position1;
                }
            }
    
            while(abs(position2)>pi)
            {
                if(position2>0)
                {
                    position2 = position2 - 2*pi;
                }else
                {
                    position2 = position2 + 2*pi;
                }
            }
            ///////////////////////////////////////////////////////

            K1 = digital_K[0];
            K2 = digital_K[1];
            K3 = digital_K[2];
            K4 = digital_K[3];       
            u = -1*(K1*position1+K2*position2+K3*velocity1+K4*velocity2);
            motor_drive(u,0);
        }

    }
}

void UpdateTheta()
{

    short EncoderPositionA = 0;
    short EncoderPositionB = 0;
    last_encoder1 = encoder1;
    last_encoder2 = encoder2;
    last_position1 = position1;
    last_position2 = position2;
    last_velocity1 = velocity1;
    last_velocity2 = velocity2;
    
    EncoderPositionA = TIM2->CNT ;
    EncoderPositionB = TIM3->CNT ;
    TIM3->CNT = 0;
    TIM2->CNT = 0;
    encoder1 +=  EncoderPositionB;
    encoder2 -=  EncoderPositionA;
        
    position1 += EncoderPositionB*(360.0f)/HALL_RESOLUTION*pi/180;
    position2 -= EncoderPositionA*(360.0f)/HALL_RESOLUTION*pi/180;
    
    position2_2 = position2;
    
    while(abs(position2_2)>pi)
    {
        if(position2_2>0)
        {
            position2_2 = position2_2 - 2*pi;
        }else
        {
            position2_2 = 2*pi + position2_2;
         }
    }
    
    velocity1 = 0.7788f*last_velocity1+EncoderPositionB*pi/4.5208f;
    velocity2 = 0.7788f*last_velocity2-EncoderPositionA*pi/4.5208f;
    

    /////////////////////theta1 is arm position2 is pendulum
}





void motor_drive(float voltA, float voltB)
{

    //you need to constraints voltage and define friction voltage
    ///////////////////////////////////////////////////////
    if(voltA>0) {
        voltA = voltA/INPUT_VOLTAGE*(INPUT_VOLTAGE-2.9f)+2.9f;
    }
    if(voltA<0) {
        voltA = voltA/INPUT_VOLTAGE*(INPUT_VOLTAGE-2.8f)-2.8f;
    }
    if(abs(voltA)>12)
    {
        voltA = voltA>0? 12:-12;
    }
    ///////////////////////////////////////////////////////

    // Convert volt to pwm
    float dutycycleA = 0.5f + voltA/(2.0f*INPUT_VOLTAGE);
    TIM1->CCR1 = CCR_value(dutycycleA);  // this command will drive motorA
}


uint16_t CCR_value(float duty)
{
    // Convert PWM duty cycle to CCR (capture/compare register) value
    return duty * uint16_t(TIM1->ARR);
}


void InitEncoder(void)
{
    // Hardware Quadrature Encoder AB for Nucleo F446RE
    // Output on debug port to host PC @ 9600 baud

    /* Connections
    PA_0 = Encoder1 A
    PA_1 = Encoder1 B
    PB_5 = Encoder2 A
    PB_4 = Encoder2 B
    */

    // configure GPIO PA0, PA1, PB5 & PB4 as inputs for Encoder
    RCC->AHB1ENR |= 0x00000003;  // Enable clock for GPIOA & GPIOB

    GPIOA->MODER   |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 ;           // PA0 & PA1 as Alternate Function  /*!< GPIO port mode register,               Address offset: 0x00      */
    GPIOA->PUPDR   |= GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR1_0 ;           // Pull Down                        /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
    GPIOA->AFR[0]  |= 0x00000011 ;                                          // AF1 for PA0 & PA1                /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
    GPIOA->AFR[1]  |= 0x00000000 ;                                          //                                  /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */


    GPIOB->MODER   |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1 ;           // PB5 & PB4 as Alternate Function  /*!< GPIO port mode register,               Address offset: 0x00      */
    GPIOB->PUPDR   |= GPIO_PUPDR_PUPDR4_0 | GPIO_PUPDR_PUPDR5_0 ;           // Pull Down                        /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
    GPIOB->AFR[0]  |= 0x00220000 ;                                          // AF2 for PB5 & PB4                /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
    GPIOB->AFR[1]  |= 0x00000000 ;                                          //                                  /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */

    // configure TIM2 & TIM3 as Encoder input
    RCC->APB1ENR |= 0x00000003;  // Enable clock for TIM2 & TIM3

    TIM2->CR1   = 0x0001;     // CEN(Counter ENable)='1'     < TIM control register 1
    TIM2->SMCR  = 0x0003;     // SMS='011' (Encoder mode 3)  < TIM slave mode control register
    TIM2->CCMR1 = 0xF1F1;     // CC1S='01' CC2S='01'         < TIM capture/compare mode register 1
    TIM2->CCMR2 = 0x0000;     //                             < TIM capture/compare mode register 2
    TIM2->CCER  = 0x0011;     // CC1P CC2P                   < TIM capture/compare enable register
    TIM2->PSC   = 0x0000;     // Prescaler = (0+1)           < TIM prescaler
    TIM2->ARR   = 0xffffffff; // reload at 0xfffffff         < TIM auto-reload register

    TIM2->CNT = 0x0000;  //reset the counter before we use it

    TIM3->CR1   = 0x0001;     // CEN(Counter ENable)='1'     < TIM control register 1
    TIM3->SMCR  = 0x0003;     // SMS='011' (Encoder mode 3)  < TIM slave mode control register
    TIM3->CCMR1 = 0xF1F1;     // CC1S='01' CC2S='01'         < TIM capture/compare mode register 1
    TIM3->CCMR2 = 0x0000;     //                             < TIM capture/compare mode register 2
    TIM3->CCER  = 0x0011;     // CC1P CC2P                   < TIM capture/compare enable register
    TIM3->PSC   = 0x0000;     // Prescaler = (0+1)           < TIM prescaler
    TIM3->ARR   = 0xffffffff; // reload at 0xfffffff         < TIM auto-reload register

    TIM3->CNT = 0x0000;  //reset the counter before we use it
}


void InitMotor(float pwm_frequency)
{
    uint16_t reload = 90000000 / int(pwm_frequency * 1000) - 1;
    uint16_t stop = 90000000 / int(pwm_frequency * 1000) / 2 - 1;

    TIM1->CR1 &= (~0x0001);   // CEN(Counter ENable) = '0'        < TIM control register 1              ;Disable counter at initial
    TIM1->PSC = 0x0001;      // Prescaler = ('1' + 1)            < TIM prescaler                       ;Prescaler timer for Timer 1
    TIM1->ARR = reload;      // reload at 180MHz/PSC/PWM freq -1 < TIM auto-reload register            ;Set auto-reload, the pwm freq is (timer_clk /PSC /ARR)
    TIM1->CCMR1 |= 0x0808;//??      // set PWM mode, preload            < TIM capture/compare mode register 1 ;Not necessary
    TIM1->CCER |= 0x0055;      // CC2NE CC2E CC1NE CC1E            < TIM capture/compare enable register ;Enable complementary PWM for channel 1, channel 2
    TIM1->BDTR |= 0x0C00;      // OSSI OSSR                        < TIM break and dead-time register    ;Set off-state selection
    TIM1->EGR = 0x0001;      // UG                               < TIM event generation register       ;Update generation
    TIM1->CR1 |= 0x0001;      // CEN(Counter ENable) = '1'        < TIM control register 1              ;Enable counter
    pc.printf("CCER : %x\r",TIM1->CCER);
    /*
        // used for debug
        pc.printf("CR1 : %d\r",uint16_t(TIM1->CR1));
        pc.printf("PSC : %d\r",uint16_t(TIM1->PSC));
        pc.printf("ARR : %d\r",uint16_t(TIM1->ARR));
        pc.printf("CCMR1 : %x\r",TIM1->CCMR1);
        pc.printf("CCER : %x\r",TIM1->CCER);
        pc.printf("BDTR : %x\r",TIM1->BDTR);
        pc.printf("EGR : %x\r",TIM1->EGR);
        pc.printf("stop : %d\r",stop);
    */
    TIM1->CCR1 = stop;
    TIM1->CCR2 = stop;
}

