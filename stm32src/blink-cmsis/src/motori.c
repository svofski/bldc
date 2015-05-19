#include "stm32f10x.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_tim.h"
#include "xprintf.h"
#include "delay.h"
#include "motori.h"
#include "qenc.h"
#include "util.h"


extern TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
extern TIM_OCInitTypeDef TIM_OCInitStructure;
extern TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
extern GPIO_InitTypeDef GPIO_InitStructure;

#define PWM_FREQ    10000
#define SCALE_BITS  8

#include "popokurve.h"

#define FULL_CIRCLE ((sizeof(popokurve)/sizeof(popokurve[0])) << SCALE_BITS)

const int32_t full_circle = FULL_CIRCLE;

// Three phases with 
#define PHASE1  (0)
#define PHASE2  ((FULL_CIRCLE/3))
#define PHASE3  ((FULL_CIRCLE/3*2))

volatile /* static */ int32_t phaseA, phaseB, phaseC;
volatile static int32_t motor_speed;
volatile static int32_t motor_power;
volatile static int enabled;
volatile static int mode;
volatile static int goal;

volatile int servostate = 0;
volatile int electric_error = 0;
volatile int electric_enc = 0;
volatile int electric_error_1 = 0;
volatile int electric_error_diff = 0;

volatile int error_1 = 0;
volatile int error_diff = 0;

volatile int servo_error;
volatile int servo_velocity;
volatile int velocity_counter = 0;
volatile int last_position = 0;

int rampcount;
int rampdown;
int start_error;

volatile int pid_Kp = 250;
volatile int pid_Ki = 20;
volatile int pid_Kd = 16;
volatile int pid_Kb = 70;


//#define MARGIN (103 - 1)
#define MARGIN 80

#define MAX_E 10000

volatile int integral_error = 0, last_error = 0, diff_error = 0;

int ctr = 0;
int pidctr = 0;


static void GPIO_EnablePWM(int enable);

#define signify(ref,x) (((ref) < 0) ? -(x) : (x))

static inline int fp_scale(int32_t value, int32_t scale)
{
    return (value * scale) >> 8;
}

static void Motori_NVIC_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the USARTx Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure);
}


void Motori_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2ENR_AFIOEN, ENABLE);

    GPIO_EnablePWM(0);

    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct); 
    TIM_TimeBaseInitStruct.TIM_Period = 255;//(SystemCoreClock / PWM_FREQ) - 1;
    TIM_TimeBaseInitStruct.TIM_Prescaler = 12; // 12 = nice and silent, 10 the motor runs too hot
    TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;//4;          !!! 4 for Ramp
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);

    /* Channel 1, 2 and 3 Configuration in PWM mode */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = TIM_TimeBaseInitStruct.TIM_Period / 2;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;    
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);     
    
    TIM_OCInitStructure.TIM_Pulse = TIM_TimeBaseInitStruct.TIM_Period / 2;
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Disable);     
     
    TIM_OCInitStructure.TIM_Pulse = TIM_TimeBaseInitStruct.TIM_Period / 2;
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Disable);     

    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
    TIM_BDTRInitStructure.TIM_DeadTime = 75;
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

    // Ensure interrupt priority over USART etc
    Motori_NVIC_Init();

    Motori_Reset();

    xprintf("SCALE_BITS=%d FULL_CIRCLE=%d\n", SCALE_BITS, FULL_CIRCLE);
}

static void GPIO_EnablePWM(int enable)
{
    if (enable) {
        // GPIO pins to alternative function push-pull: PA8 PA9 PA10  inv: PB13 PB14 PB15
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &GPIO_InitStructure);    
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
        GPIO_Init(GPIOB, &GPIO_InitStructure);    
    } else {
        // Reset GPIO to regular ouputs and output low
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &GPIO_InitStructure);    
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
        GPIO_Init(GPIOB, &GPIO_InitStructure);

        GPIOA->BRR = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
        GPIOB->BRR = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    }
}

// Abort everything, ensure that MOSFETs are turned off
void Motori_Halt(void)
{
    NVIC_DisableIRQ(TIM1_UP_IRQn);
    TIM_Cmd(TIM1, DISABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, DISABLE);
    GPIO_EnablePWM(0);
}

void Motori_Reset(void)
{
    phaseA = PHASE1;
    phaseB = PHASE2;
    phaseC = PHASE3;
    motor_speed = 0;
    motor_power = 150;
    enabled = 0;
}

void Motori_Enable(int enable) {
    if (enable) {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
        TIM_CtrlPWMOutputs(TIM1, ENABLE);
        TIM_Cmd(TIM1, ENABLE);
        NVIC_EnableIRQ(TIM1_UP_IRQn);
        GPIO_EnablePWM(1);
        enabled = 1;
    } else {
        TIM_CtrlPWMOutputs(TIM1, DISABLE);
        TIM_Cmd(TIM1, DISABLE);
        NVIC_DisableIRQ(TIM1_UP_IRQn);
        GPIO_EnablePWM(0);
        enabled = 0;
    }
}

int Motori_GetEnabled(void) {
    return enabled;
}

void Motori_SetPower(int power) {
    servostate = 0;
    motor_power = power;
}

int Motori_GetPower(void) {
    return motor_power;
}

void Motori_SetSpeed(int speed)
{
    motor_speed = speed;
}

int Motori_GetSpeed(void)
{
    return motor_speed;
}

int Motori_GetElectricalAngle(void)
{
    return phaseA >> SCALE_BITS;
}

void Motori_SetElectricalAngleDegrees(int angle)
{
    // xprintf("SetEA: angle=%d angle %% 360=%d scaled=%d, offset=%d reset=%d fullcircled=%d", 
    //     angle, 
    //     angle % 360, 
    //     (angle % 360) << SCALE_BITS, 
    //     ((angle % 360) << SCALE_BITS) + full_circle,
    //     (((angle % 360) << SCALE_BITS) + full_circle) >> SCALE_BITS,
    //     ((angle % 360) << SCALE_BITS) % FULL_CIRCLE);
    Motori_SetElectricalAngle(((angle % 360) << SCALE_BITS) + FULL_CIRCLE);
    // xprintf(" phaseA=%d\n", phaseA);
}

void Motori_SetElectricalAngle(int angle)
{
    //motor_speed = 0;
    __disable_irq();
    phaseA = angle % FULL_CIRCLE;
    if (phaseA < 0) {
        phaseA += full_circle;
    }
    phaseB = phaseA + PHASE2;
    if (phaseB >= full_circle) {
        phaseB -= full_circle;
    }
    phaseC = phaseA + PHASE3;
    if (phaseC >= full_circle) {
        phaseC -= full_circle;
    }
    __enable_irq();
}

// x = 0..99 -> 0..255
// 

int Motori_GetEAError(int quadrature_count)
{
    int qcmod = quadrature_count % 103;
    qcmod = (qcmod << SCALE_BITS) / 103;
    return (360 * qcmod - phaseA) >> SCALE_BITS;
}

int Motori_GetEAError2(int quadrature_count)
{
    int phase;
    __disable_irq();
    phase = phaseA;
    __enable_irq();
    int qcmod = (quadrature_count % 720) % 103;
    qcmod = (qcmod << SCALE_BITS) / 103;
    int err = (360 * qcmod - phase) >> SCALE_BITS;
    if (err > 180) {
        err = 360 - err;
    } else if (err < -180) {
        err = -360 - err;
    }
    if (err > 180 || err < -180) {
        xprintf("WTF: err=%d phase=%d qenc=%d qcmod=%d\n", err, phase, quadrature_count, qcmod);
    }
    return err;
}

void Motori_SetMode(const MotorMode newmode)
{
    mode = newmode;
}

void Motori_SetPosition(int position)
{
    goal = position;
    servostate = -1;
    rampcount = 0;
    start_error = goal - QEnc_GetPosition();
    mode = SERVO;
}


int32_t PID(int32_t error) 
{
    integral_error = (int32_t)integral_error + (int32_t)error;

    if (ctr == 0) {
        ctr = 512;
        diff_error = error - last_error;
        last_error = error;
    } 
    --ctr;

    return (error) * pid_Kp + (integral_error * pid_Ki >> (SCALE_BITS + 4)) + (diff_error) * pid_Kd;
}

#define MAX_SPEED 250

#define CONST_SPEED 40

int brake_start, top_speed;

//static inline int fp_scale(int32_t value, int32_t scale)
// {
//     return (value * scale) >> 8;
// }

static inline void motor_Tick_Servo();
static inline void motor_Tick_Servo_Ramp();
static inline void motor_Tick_Normal();

static inline void motor_Tick_Servo_Angular()
{
    int power = motor_power;
    int position = QEnc_GetPosition();
    static int zero_count = 0;

    if (position != last_position) {
        // velocity = 1/velocity_counter
        servo_velocity = (servo_velocity + velocity_counter) / 2;
        last_position = position;
        velocity_counter = 0;
    } else {
        velocity_counter++;
    }

    servo_error = goal - position;
    if (velocity_counter < 1000) {
        power = 255;
    }

    if (servostate == -1) {
        // if (abs(servo_error) > 120)
        //     pid_Kp = 240; // 250
        // else
        //     pid_Kp = 270;
        if (abs(servo_error) < 32)
            servostate = 2;
        else
            servostate = 0;
    }


    // An attempt to smoothly gain the torque at the end of the path
    // works well but the oscillations tend to be very long

    // if (abs(servo_error) < 4) {
    //     motor_speed = servo_error * 3/2; 
    //     motor_Tick_Normal();
    //     return;
    // } 

    if (abs(servo_error) < 3 || servostate == 2) {
        motor_speed = servo_error * 3/2;
        motor_Tick_Normal();
        return;
    } 

    electric_error = (servo_error * 7) / 2;
    int electric_position = (position * 7) / 2;

    int add = (electric_error * pid_Kp) >> SCALE_BITS;
    
    if (add < -pid_Kb)
        add = -pid_Kb;
    else if (add > pid_Kb)
        add = pid_Kb;

    // Braking by velocity works really well but tends to create brutal kicks
    servostate = 0;
    if (abs(servo_error) < 80 && servo_velocity < pid_Kd) {
        int sub = (pid_Ki * 16) / servo_velocity;
        if (sub > 80) sub = 80;
        servostate = signify(-add, sub);
        add += signify(-add, sub);
    }


    int new_electric = electric_position + add;
    if (new_electric < 0) {
         new_electric = (new_electric % 360) + 360;
    }
    if (new_electric < 0) {//} || new_electric >= 360) {
        Motori_Halt();
        __asm("BKPT #0");
    }
    Motori_SetElectricalAngle(new_electric << SCALE_BITS);

    int chan1 = fp_scale(popokurve[phaseA >> SCALE_BITS] - 128, power) + 128;
    int chan2 = fp_scale(popokurve[phaseB >> SCALE_BITS] - 128, power) + 128;
    int chan3 = fp_scale(popokurve[phaseC >> SCALE_BITS] - 128, power) + 128;

    TIM_SetCompare1(TIM1, chan1);
    TIM_SetCompare2(TIM1, chan2);
    TIM_SetCompare3(TIM1, chan3);
}

static inline void motor_Tick_Servo_PID_Speed()
{
    int power = motor_power;
    int position = QEnc_GetPosition();
    static int step;
    static int zero_count = 0;
    int error = goal - position;

    // electric_error = error * 7/2
    electric_error = (error * 7) / 2;

    if (error == 0) {
        motor_speed = 0;
    }
    else 
    {
        power = 255;
        motor_speed = PID(electric_error) >> (SCALE_BITS + 1);
        if (motor_speed > MAX_SPEED)
            motor_speed = MAX_SPEED;
        else if (motor_speed < -MAX_SPEED)
            motor_speed = -MAX_SPEED;
    }

    int phase = phaseA + motor_speed;
    if (phase < 0) {
        phase += full_circle;
    }
    
    Motori_SetElectricalAngle(phase);

    int chan1 = fp_scale(popokurve[phaseA >> SCALE_BITS] - 128, power) + 128;
    int chan2 = fp_scale(popokurve[phaseB >> SCALE_BITS] - 128, power) + 128;
    int chan3 = fp_scale(popokurve[phaseC >> SCALE_BITS] - 128, power) + 128;

    TIM_SetCompare1(TIM1, chan1);
    TIM_SetCompare2(TIM1, chan2);
    TIM_SetCompare3(TIM1, chan3);
}

static inline void motor_Tick_Servo_Ramp()
{
    int power = motor_power;
    int position = QEnc_GetPosition();
    static int step;
    static int zero_count = 0;

    int error = goal - position;
    // if (error == 0) {
    //     motor_speed = 0;
    //     servostate = 3;
    // }

    // 0: ramp up
    // 1: const
    // 2: ramp down
    switch(servostate) {
        case -1:
            start_error = error;
            servostate = 0;
            rampcount = 0;
            step = 8;
            motor_speed = signify(error, CONST_SPEED);

            {   
                int path = abs(start_error);
                // if (path <= 31) {
                //     motor_speed = signify(error, 4); // 40 is ideal for <= 15, but 80 works better for 4-30
                //     servostate = 5;
                // }
                // else 
                if (path <= 120) {                    
                    top_speed = signify(error, CONST_SPEED);
                    brake_start = -1; 
                    servostate = 4;
                } else {
                    top_speed = signify(error, MAX_SPEED);
                    brake_start = abs(start_error - start_error / 5);
                    motor_speed = signify(error, CONST_SPEED);
                    servostate = 7;
                }
            }
            break;
        case 7:
            rampcount++;
            if (error > 0)
                motor_speed += 5;
            else
                motor_speed -= 5;                      
            if (abs(error) < brake_start) {
                servostate = 9;
            }
            break;
#define KP9 9
        case 9:
            if (motor_speed > 0)
                 motor_speed -= 1;
            else if (motor_speed < 0)
                 motor_speed += 1;
            if (abs(abs(error * KP9) - abs(motor_speed)) < 10)
                servostate = 10;
            break;

        case 10:
            motor_speed = error * KP9;
            if (error != 0) {
                zero_count = 512;
            }
            break;

        case 3:
            motor_speed = error * 2;   //11
            if (error != 0) {
                zero_count = 512;
            }
            break;
        case 4:
            // const speed, brake
            if (abs(error) < 14)        //15
                servostate = 3;
            else if (abs(error) < 50)
                motor_speed = signify(error, CONST_SPEED/2);
            break;
        case 5:
            if (abs(error) <= 7)
                servostate = 6;
            break;
        case 6:
            motor_speed = error * 1;
            if (error != 0) {
                zero_count = 512;
            }
            break;
        default:
            break;
    }

    power = 255;
    if (error == 0) {
        if (zero_count > 0) zero_count--;
        if (zero_count == 0)
            power = motor_power;
    }

    int phase = phaseA + motor_speed;
    if (phase < 0) {
        phase += full_circle;
    }
    
    Motori_SetElectricalAngle(phase);

    int chan1 = fp_scale(popokurve[phaseA >> SCALE_BITS] - 128, power) + 128;
    int chan2 = fp_scale(popokurve[phaseB >> SCALE_BITS] - 128, power) + 128;
    int chan3 = fp_scale(popokurve[phaseC >> SCALE_BITS] - 128, power) + 128;

    TIM_SetCompare1(TIM1, chan1);
    TIM_SetCompare2(TIM1, chan2);
    TIM_SetCompare3(TIM1, chan3);
}


static inline void motor_Tick_Servo_PID()
{
    int power = motor_power;
    int position = QEnc_GetPosition();

    int error = goal - position;
    if (error == 0) {
        motor_speed = 0;
    }
    else 
    {
        int want_speed = PID(error) >> SCALE_BITS;
        if (want_speed > MAX_SPEED) 
            want_speed = MAX_SPEED;
        if (want_speed < -MAX_SPEED)
            want_speed = -MAX_SPEED;

        if (abs(want_speed - motor_speed) > 80) {
            if (want_speed > 0)
                motor_speed += 7;
            else 
                motor_speed -= 7;
        } else
            motor_speed = want_speed;
    }

    if (abs(motor_speed) > 10)
        power = 255;

    if (abs(error) < 4)
        motor_speed /= 2;


    int phase = phaseA + motor_speed;
    if (phase < 0) {
        phase += full_circle;
    }
    
    Motori_SetElectricalAngle(phase);

    int chan1 = fp_scale(popokurve[phaseA >> SCALE_BITS] - 128, power) + 128;
    int chan2 = fp_scale(popokurve[phaseB >> SCALE_BITS] - 128, power) + 128;
    int chan3 = fp_scale(popokurve[phaseC >> SCALE_BITS] - 128, power) + 128;

    TIM_SetCompare1(TIM1, chan1);
    TIM_SetCompare2(TIM1, chan2);
    TIM_SetCompare3(TIM1, chan3);
}


static inline void motor_Tick_Servo()
{
    int speed = motor_speed;
    int power = motor_power;
    int position = QEnc_GetPosition();

    // 0 10 error = -10
    // 0 -10 error = 10

    int error = goal - position;

    if (error == 0)
        return;

    int degrees = 360 * (position % 103) / 103;

    // difference between actual position and flux vector
    electric_enc = ((position << (SCALE_BITS-1)) * 7) % FULL_CIRCLE;
    electric_error = electric_enc - phaseA;

    // electric_error_diff = abs(electric_error) - abs(electric_error_1);
    // electric_error_1 = electric_error;

    error_diff = abs(error) - abs(error_1);
    error_1 = error;

    switch (servostate) {
    case 0:
        servostate = 3;
        break;
    case 1:
        // error was <= -MARGIN
        degrees -= (error < -MARGIN * 2) ? 90 : 60;
        Motori_SetElectricalAngleDegrees(degrees);
        if (error > -MARGIN) {
            servostate = 3;
        }
        motor_speed = -2000;
        break;
    case 2:
        // error was >= MARGIN
        degrees += (error > MARGIN * 2) ? 90 : 60;
        Motori_SetElectricalAngleDegrees(degrees);
        if (error < MARGIN) {
            servostate = 3;
        }
        motor_speed = 2000;
        break;
    case 3:
        if (error <= -MARGIN)
            servostate = 1;
        else if (error >= MARGIN)
            servostate = 2;
        int step = error * 14;
        motor_speed += step;
        Motori_SetElectricalAngle(phaseA + motor_speed);
        break;
    }

    int chan1 = fp_scale(popokurve[phaseA >> SCALE_BITS] - 128, power) + 128;
    int chan2 = fp_scale(popokurve[phaseB >> SCALE_BITS] - 128, power) + 128;
    int chan3 = fp_scale(popokurve[phaseC >> SCALE_BITS] - 128, power) + 128;

    TIM_SetCompare1(TIM1, chan1);
    TIM_SetCompare2(TIM1, chan2);
    TIM_SetCompare3(TIM1, chan3);
    
}


static inline void motor_Tick_Normal()
{
    int speed = motor_speed;
    int power = motor_power;

    phaseA += speed;
    if (phaseA >= full_circle) {
        phaseA -= full_circle;
    } else if (phaseA < 0) {
        phaseA += full_circle;
    }

    phaseB += speed;
    if (phaseB >= full_circle) {
        phaseB -= full_circle;
    } else if (phaseB < 0) {
        phaseB += full_circle;
    }

    phaseC += speed;
    if (phaseC >= full_circle) {
        phaseC -= full_circle;
    } else if (phaseC < 0) {
        phaseC += full_circle;
    }

    // if ((phaseA >> SCALE_BITS) > sizeof(popokurve)) __asm("BKPT #0\n");
    // if ((phaseB >> SCALE_BITS) > sizeof(popokurve)) __asm("BKPT #0\n");
    // if ((phaseC >> SCALE_BITS) > sizeof(popokurve)) __asm("BKPT #0\n");

    int chan1 = fp_scale(popokurve[phaseA >> SCALE_BITS] - 128, power) + 128;
    int chan2 = fp_scale(popokurve[phaseB >> SCALE_BITS] - 128, power) + 128;
    int chan3 = fp_scale(popokurve[phaseC >> SCALE_BITS] - 128, power) + 128;

    TIM_SetCompare1(TIM1, chan1);
    TIM_SetCompare2(TIM1, chan2);
    TIM_SetCompare3(TIM1, chan3);
}

void TIM1_UP_IRQHandler(void)
{
    switch (mode) {
        case NORMAL:
            motor_Tick_Normal();
            break;
        case SERVO:
            motor_Tick_Servo_Angular();
            break;
    }
    TIM1->SR &= ~TIM_SR_UIF;
}