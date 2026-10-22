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
#define PWM_NOMINAL     2500
#define FWD_DUTY        PWM_NOMINAL
#define STEER_FAST      PWM_NOMINAL
#define STEER_SLOW      (PWM_NOMINAL / 2)
#define PIVOT_DUTY      PWM_NOMINAL


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


// Reflection Sensors State Readout Function
uint8_t Detect_Line_Position(uint8_t reflectance_sensor_data){
    
    switch(reflectance_sensor_data){
        
        case 0x18:
        printf("Center\n");
        LED1_Output(RED_LED_OFF);
        LED2_Output(RGB_LED_GREEN);
        break;
        
        case 0x1C:
        printf("Slightly Left from Center\n");
        LED1_Output(RED_LED_OFF);
        LED2_Output(RGB_LED_YELLOW);
        break;
        
        case 0x38:
        printf("Slightly Right from Center\n");
        LED1_Output(RED_LED_OFF);
        LED2_Output(RGB_LED_PINK);
        break;
                
                
        case 0x01:
        printf("Far Left from Center\n");
        LED1_Output(RED_LED_OFF);
        LED2_Output(RGB_LED_WHITE);
        break;

        case 0x80:
        printf("Far Right from Center\n");
        LED1_Output(RED_LED_OFF);
        LED2_Output(RGB_LED_SKY_BLUE);
        break;
        
                
        case 0xF8:
        printf("Left Turn\n");
        LED1_Output(RED_LED_ON);
        LED2_Output(RGB_LED_WHITE);
        break;
        
        
        case 0x1F:
        printf("Right Turn\n");
        LED1_Output(RED_LED_ON);
        LED2_Output(RGB_LED_SKY_BLUE);
        break;
                
                
        case 0xFF:
        printf("T-Intersection\n");
        LED1_Output(RED_LED_ON);
        LED2_Output(RGB_LED_BLUE);
        break;
                
        
        case 0x00:
        printf("Dead End \n");
        LED1_Output(RED_LED_ON);
        LED2_Output(RGB_LED_RED);
        break;

        
        default:
        printf("Unknown sensor pattern: 0x%02X\n",reflectance_sensor_data);
        LED1_Output(RED_LED_OFF);
        LED2_Output(RGB_LED_OFF);
        break;
    
    }
    
   return reflectance_sensor_data;
}


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
    }

}

// Need to add switch cases functiuons for the followinf

//Controller functuin that deterimne the state of the robot
//Line following function that controls the mottor s
//Functions that make sure we dont get false state switch for unkown sates, or T instersections


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
    EnableInterrupts();
    
    while(1)
    {

    }
}
