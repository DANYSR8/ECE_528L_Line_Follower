/**
 * @file main.c
 *
 * @brief Main source code for the Line_Follower program.
 *
 * This file contains the main entry point for the Line_Follower program.
 * The main controller demonstrates a Line Follower robot without using an advanced algorithm.
 *
 * It interfaces the following peripherals using GPIO to demonstrate line following:
 *  - 8-Channel QTRX Sensor Array module
 *
 * Timers are used in this lab:
 *  - SysTick:  Used to generate periodic interrupts at a specified rate (1 kHz)
 *  - Timer A0: Used to generate PWM signals that will be used to drive the DC motors
 *  - Timer A1: Used to generate periodic interrupts at a specified rate (1 kHz)
 *
 * @note For more information regarding the 8-Channel QTRX Sensor Array module,
 * refer to the product page: https://www.pololu.com/product/3672
 *
 * @author
 *
 */

// Initialzation of Headers
#include "msp.h"
#include "inc/Clock.h"
#include "inc/CortexM.h"
#include "inc/GPIO.h"
#include "inc/EUSCI_A0_UART.h"
#include "inc/Motor.h"
#include "inc/Timer_A1_Interrupt.h"
#include "inc/SysTick_Interrupt.h"
#include "inc/Reflectance_Sensor.h"

// Initalixzation of Constant Values
#define PWM_NOMINAL             2000
#define FWD_DUTY                PWM_NOMINAL
#define STEER_GENTLE            ((3 * PWM_NOMINAL) / 5)
#define STEER_HARD              ((3 * PWM_NOMINAL) / 20)
#define PIVOT_DUTY              PWM_NOMINAL
#define T_SEC_REAL              5
#define DEAD_END_REAL           25


static uint8_t t_count = 0;
static uint8_t DeadEnd_count = 0;


// Definitions for Reflectance Sensor State
typedef enum
{
    CENTER                 = 0b00011000,
    SLIGHT_LEFT_CENTER     = 0b00011100,
    SLIGHT_RIGHT_CENTER    = 0b00111000,
    FAR_LEFT               = 0b00000001,
    FAR_RIGHT              = 0b10000000,
    LEFT_TURN              = 0b11111000,
    RIGHT_TURN             = 0b00011111,
    T_INTERSECTION         = 0b11111111,
    DEAD_END               = 0b00000000
} Line_Position;


typedef enum
{
    FOLLOWING_PATH,
    FINDING_PATH,
    RETURNING,
    STOPPED,

} Robot_State;


// Initalization of Global Varibles
static uint32_t Timer_A1_ms_elapsed = 0;
Robot_State Current_Robot_State = FOLLOWING_PATH ;


/**
 * @brief Updates the consecutive-reading counters for the T-intersection and dead end.
 *
 * Each counter increases by one while its pattern is read on consecutive
 * samples and resets to zero on any other reading. The controller compares
 * these counts against T_SEC_REAL and DEAD_END_REAL so that a single noisy
 * reading cannot trigger a turnaround or a stop.
 *
 * @param reflectance_sensor_data 8-bit reading from the sensor array.
 *
 * @return None
 */
void update_Counters(uint8_t reflectance_sensor_data)
{
    if (reflectance_sensor_data == T_INTERSECTION)
    {
        t_count = t_count + 1;
    }
    else
    {
        t_count = 0;
    }

    if (reflectance_sensor_data == DEAD_END)
    {
        DeadEnd_count = DeadEnd_count + 1;
    }
    else
    {
        DeadEnd_count = 0;
    }
}


// Reflection Sensors State Readout Function
/**
 * @brief Shows the detected line position on the LEDs.
 *
 * Compares the raw sensor reading against the known Line_Position patterns
 * and sets LED1 and the RGB LED (LED2) as a debug indicator. LED1 turns on
 * for turns, the T-intersection, and dead end. This function does not drive
 * the motors.
 *
 * @param reflectance_sensor_data 8-bit reading from the sensor array.
 *
 * @return The same sensor reading, unchanged.
 */
uint8_t Detect_Line_Position(uint8_t reflectance_sensor_data){

    switch(reflectance_sensor_data){

        case CENTER:
        LED1_Output(RED_LED_OFF);
        LED2_Output(RGB_LED_GREEN);
        break;

        case SLIGHT_LEFT_CENTER:
        LED1_Output(RED_LED_OFF);
        LED2_Output(RGB_LED_YELLOW);
        break;

        case SLIGHT_RIGHT_CENTER:
        LED1_Output(RED_LED_OFF);
        LED2_Output(RGB_LED_PINK);
        break;

        case FAR_LEFT:
        LED1_Output(RED_LED_OFF);
        LED2_Output(RGB_LED_WHITE);
        break;

        case FAR_RIGHT:
        LED1_Output(RED_LED_OFF);
        LED2_Output(RGB_LED_SKY_BLUE);
        break;

        case LEFT_TURN:
        LED1_Output(RED_LED_ON);
        LED2_Output(RGB_LED_WHITE);
        break;

        case RIGHT_TURN:
        LED1_Output(RED_LED_ON);
        LED2_Output(RGB_LED_SKY_BLUE);
        break;

        case T_INTERSECTION:
        LED1_Output(RED_LED_ON);
        LED2_Output(RGB_LED_BLUE);
        break;

        case DEAD_END:
        LED1_Output(RED_LED_ON);
        LED2_Output(RGB_LED_RED);
        break;

        default:
        LED1_Output(RED_LED_OFF);
        LED2_Output(RGB_LED_OFF);
        break;

    }

   return reflectance_sensor_data;
}


/**
 * @brief Drives the motors to keep the robot centered on the line.
 *
 * Updates the debounce counters, then picks a motor command from the reading
 * using four levels of correction. The outer wheel never runs above FWD_DUTY:
 *  - Centered:                   both wheels at FWD_DUTY
 *  - Slightly off-center:        inner wheel at STEER_GENTLE
 *  - Well off-center or a turn:  inner wheel at STEER_HARD
 *  - Line at the outer sensors:  pivot in place at PIVOT_DUTY
 *
 * The T-intersection, dead end, and any unknown pattern send no new command,
 * so the robot keeps its last correction. Decisions about the T-intersection
 * and dead end are made in Controller().
 *
 * @param reflectance_sensor_data 8-bit reading from the sensor array.
 *
 * @return None
 */
void LineFollowing(uint8_t reflectance_sensor_data){

    update_Counters(reflectance_sensor_data);

    switch(reflectance_sensor_data){

    case CENTER:
        Motor_Forward(FWD_DUTY, FWD_DUTY);
        break;

    case SLIGHT_LEFT_CENTER:
        Motor_Forward(FWD_DUTY, STEER_GENTLE);
        break;

    case SLIGHT_RIGHT_CENTER:
        Motor_Forward(STEER_GENTLE, FWD_DUTY);
        break;

    case FAR_LEFT:
    case 0x03:
        Motor_Right(PIVOT_DUTY, PIVOT_DUTY);
        break;

    case FAR_RIGHT:
    case 0xC0:
        Motor_Left(PIVOT_DUTY, PIVOT_DUTY);
        break;

    case RIGHT_TURN:
    case 0x06:
    case 0x07:
    case 0x0C:
        Motor_Forward(FWD_DUTY, STEER_HARD);
        break;

    case LEFT_TURN:
    case 0x60:
    case 0xE0:
    case 0x30:
        Motor_Forward(STEER_HARD, FWD_DUTY);
        break;

    case DEAD_END:
    case T_INTERSECTION:
    default:
        break;

    }

}


/**
 * @brief State machine that decides what the robot does on each sensor reading.
 *
 * Called once per sensor reading from Timer_A1_Periodic_Task().
 *  - FOLLOWING_PATH: follow the line. After T_SEC_REAL consecutive
 *    T-intersection reads, reset the counters, start spinning, and switch
 *    to FINDING_PATH.
 *  - FINDING_PATH: spin in place until the line is centered, then reset the
 *    counters and switch to RETURNING.
 *  - RETURNING: follow the line back. After DEAD_END_REAL consecutive dead
 *    end reads, stop the motors and switch to STOPPED.
 *  - STOPPED: keep the motors off.
 *
 * @param reflectance_sensor_data 8-bit reading from the sensor array.
 *
 * @return None
 */
void Controller(uint8_t reflectance_sensor_data)
{

    switch (Current_Robot_State)
    {
        case FOLLOWING_PATH:

            if(t_count >= T_SEC_REAL)
            {
                t_count = 0 ;
                DeadEnd_count = 0;
                Motor_Right(PIVOT_DUTY, PIVOT_DUTY);
                Current_Robot_State = FINDING_PATH;
            }

            else
            {
                LineFollowing(reflectance_sensor_data);
            }

            break;


        case FINDING_PATH:

            if (reflectance_sensor_data == CENTER)
            {
                t_count = 0 ;
                DeadEnd_count = 0;
                Motor_Forward(FWD_DUTY, FWD_DUTY);      // line found, head back
                Current_Robot_State = RETURNING;
            }

            else
            {
                Motor_Right(PIVOT_DUTY, PIVOT_DUTY);    // keep spinning
            }
            break;


        case RETURNING:

            if(DeadEnd_count >= DEAD_END_REAL)
            {
                t_count = 0 ;
                DeadEnd_count = 0;
                Motor_Stop();
                Current_Robot_State = STOPPED;
            }

            else
            {
                LineFollowing(reflectance_sensor_data);
            }

            break;


        case STOPPED:
        default:
            Motor_Stop();
            Current_Robot_State = STOPPED;
            break;
    }
}



/**
 * @brief Periodic task called by the Timer A1 interrupt every 1 ms.
 *
 * Every 10 ms it starts a reflectance sensor measurement, then on the next
 * tick reads the result, updates the LEDs, and runs the controller.
 *
 * @return None
 */
void Timer_A1_Periodic_Task(void)
{

Timer_A1_ms_elapsed++;

    if ((Timer_A1_ms_elapsed % 10) == 0)
    {
     Reflectance_Sensor_Start();
    }

    if((Timer_A1_ms_elapsed % 10) == 1)
    {
        uint8_t Reflectance_Sensor_Data = Reflectance_Sensor_End();
        Detect_Line_Position(Reflectance_Sensor_Data);
        Controller(Reflectance_Sensor_Data);
    }

}


int main(void)
{

    DisableInterrupts();
    Clock_Init48MHz();
    LED1_Init();
    LED2_Init();
    EUSCI_A0_UART_Init_Printf();
    Reflectance_Sensor_Init();
    SysTick_Interrupt_Init(SYSTICK_INT_NUM_CLK_CYCLES,SYSTICK_INT_PRIORITY);
    Timer_A1_Interrupt_Init(&Timer_A1_Periodic_Task,TIMER_A1_INT_CCR0_VALUE);
    Motor_Init();
    EnableInterrupts();


    while(1)
    {

    }
}
