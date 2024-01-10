/*
 * Tamagotchi project
 * Course: Introduction to the Computer Systems at University of Oulu
 *
 * Atte Kiviniemi, Teemu Kolu
 *
 * */

/* C Standard library */
#include <stdio.h>

/* XDCtools files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/UART.h>

/* Board Header files */
#include "Board.h"
#include "sensors/mpu9250.h"
#include "buzzer.h"

/* Task */
#define STACKSIZE 2048
Char mpuTaskStack[STACKSIZE];
Char uartTaskStack[STACKSIZE];

//RTOS variables
static PIN_Handle hMpuPin;
static PIN_State MpuPinState;
static PIN_Handle ledHandle;
static PIN_State ledState;
static PIN_Handle buttonHandle;
static PIN_State buttonState;
static PIN_Handle hBuzzer;
static PIN_State sBuzzer;

// Define a structure to hold the limit values
struct Limits {
    float max_ax;
    float laying_ax;
    float max_gz;
};

// MPU power pin
static PIN_Config MpuPinConfig[] = {
    Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

// PIN configurations for both buttons
PIN_Config buttonConfig[] = {
   Board_BUTTON0 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
   Board_BUTTON1 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
   PIN_TERMINATE // Terminate the configuration array with this constant
};

// PIN configurations for both leds
PIN_Config ledConfig[] = {
   Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
   Board_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
   PIN_TERMINATE // Terminate the configuration array with this constant
};

// Buzzer configurations
PIN_Config cBuzzer[] = {
  Board_BUZZER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};

// MPU uses its own I2C interface
static const I2CCC26XX_I2CPinCfg i2cMPUCfg = {
    .pinSDA = Board_I2C0_SDA1,
    .pinSCL = Board_I2C0_SCL1
};

/* Finite State Machine */
enum stateProgram {READ_SENSOR = 1, UPDATE, NEW_MSG, RUNAWAY, SLEEP};
enum stateActivity {ACTIVATE = 1, EAT, EXERCISE, PET};

enum stateProgram programState = READ_SENSOR;
enum stateActivity activityState = ACTIVATE;

// Function prototypes
void buzzerSound(int num);
void delay(unsigned int);
char *strstr(const char *haystack, const char *needle);
size_t strlen(const char *str);

void buttonFxn(PIN_Handle handle, PIN_Id pinId) {
    if (pinId == Board_BUTTON0) {
        // When button0 is pressed
        if (programState == SLEEP) {
            // change programState SLEEP to READ_SENSOR
            programState = READ_SENSOR;
        }
    }

    if (pinId == Board_BUTTON1) {
        // When button1 is pressed
        if(PIN_getOutputValue(Board_LED1) == 0 && PIN_getOutputValue(Board_LED2) == 0){
            // Turn red led ON and activityState to EAT
            PIN_setOutputValue(ledHandle, Board_LED1, 1);
            activityState = EAT;
        } else if (PIN_getOutputValue(Board_LED1) == 1 && PIN_getOutputValue(Board_LED2) == 0) {
            // Turn red led OFF and green led ON and activityState to EXERCISE
            PIN_setOutputValue(ledHandle, Board_LED2, 1);
            PIN_setOutputValue(ledHandle, Board_LED1, 0);
            activityState = EXERCISE;
        } else if (PIN_getOutputValue(Board_LED1) == 0 && PIN_getOutputValue(Board_LED2) == 1) {
            // Turn red led ON and activityState to PET (Now both leds are ON)
            PIN_setOutputValue(ledHandle, Board_LED1, 1);
            activityState = PET;
        } else if (PIN_getOutputValue(Board_LED1) == 1 && PIN_getOutputValue(Board_LED2) == 1) {
            // Turn both leds OFF and activityState to ACTIVATE
            PIN_setOutputValue(ledHandle, Board_LED1, 0);
            PIN_setOutputValue(ledHandle, Board_LED2, 0);
            activityState = ACTIVATE;
        }
    }
}

void marioTheme() {

    /*
     *  Imitating Super Mario Bros. theme song
     *  Got note frequency and delay time from https://gist.github.com/gskielian/6135641
     *  Replaced tone() fuction with buzzerSetFrequency() and looped song
     *
     */

    // Define note frequencies
    int noteLoop[] = {510, 380, 320, 440, 480, 450, 430, 380, 660, 760, 860, 700, 760, 660, 520, 580, 480};
    int noteIntro[] = {660, 660, 660, 510, 660, 770, 380};
    // Define note durations
    int durationIntro[] = {150, 300, 300, 100, 300, 550, 575};
    int durationLoop[] = {450, 400, 500, 300, 330, 150, 300, 200, 200, 150, 300, 150, 350, 300, 150, 150, 500};
    int i;

    buzzerOpen(hBuzzer);

    for (i = 0; i < sizeof(noteIntro) / sizeof(noteIntro[0]); i++) {
        if (programState != SLEEP) {
            buzzerClose();
            return;
        }

        buzzerSetFrequency(noteIntro[i]);
        delay(durationIntro[i]);
    }

    while (programState == SLEEP) {
        for (i = 0; i < sizeof(noteLoop) / sizeof(noteLoop[0]); i++) {
            if (programState != SLEEP) {
                buzzerClose();
                return;
            }

            buzzerSetFrequency(noteLoop[i]);
            delay(durationLoop[i]);
        }
    }
    buzzerClose();
}

void buzzerSound(int num){

    buzzerOpen(hBuzzer);

    if (num == 1) {
        // Right movement detected (Imitating video game coin sound)
        buzzerSetFrequency(987.77);
        delay(50);
        buzzerSetFrequency(1218.51);
        delay(200);

    } else if (num == 2) {
        buzzerSetFrequency(700);
        delay(50);
        buzzerSetFrequency(676);
        delay(50);
        buzzerSetFrequency(500);
        delay(50);

    } else if (num == 3) {
        // Notification sound
        buzzerSetFrequency(1047);
        delay(100);
        buzzerSetFrequency(1319);
        delay(100);
        buzzerSetFrequency(1568);
        delay(100);
        buzzerSetFrequency(2093);
        delay(50);

    } else if (num == 4) {
        // Game over - tamagotchi ran away
        buzzerSetFrequency(523);
        delay(450);
        buzzerSetFrequency(392);
        delay(450);
        buzzerSetFrequency(330);
        delay(300);

        buzzerSetFrequency(440);
        delay(225);
        buzzerSetFrequency(494);
        delay(225);
        buzzerSetFrequency(440);
        delay(225);
        buzzerSetFrequency(415);
        delay(225);
        buzzerSetFrequency(466);
        delay(225);
        buzzerSetFrequency(415);
        delay(225);
        buzzerSetFrequency(392);
        delay(150);
        buzzerSetFrequency(294);
        delay(150);
        buzzerSetFrequency(330);
        delay(900);
    }
    buzzerClose();
}

int movement(float arr_ax[], float arr_gz[], int len) {
    // Limit values
    struct Limits limit = {0.64475, 0.952375, 51.60522};

    // Sum, max and variance variables
    float sum_ax = 0.0, sum_gz = 0.0;
    float max_ax = 0.0, max_gz = 0.0;
    int i;

    // Sum values from array and find max value for acceleration x and gyro z
    for (i = 0; i < len; i++) {
        sum_ax += arr_ax[i];
        sum_gz += arr_gz[i];

        if (arr_ax[i] > max_ax) {
            max_ax = arr_ax[i];
        }

        if (arr_gz[i] > max_gz) {
            max_gz = arr_gz[i];
        }
    }

    // Calculate the mean
    float mean_ax = sum_ax / len;

    // Check if the conditions are met for the right movements
    if (max_ax > limit.max_ax && max_gz > limit.max_gz) {
        return 1;
    } else if (max_ax < limit.max_ax && max_gz > limit.max_gz) {
        return 2;
    } else if (mean_ax > limit.laying_ax) {
        return 3;
    }

    return 0;
}

void mpuTask(UArg arg0, UArg arg1) {

    float ax, ay, az, gx, gy, gz;
    int index = 0;
    float arr_ax[19], arr_gz[19];

    I2C_Handle i2cMPU; // Own i2c-interface for MPU9250 sensor
    I2C_Params i2cMPUParams;

    I2C_Params_init(&i2cMPUParams);
    i2cMPUParams.bitRate = I2C_400kHz;
    // Note the different configuration below
    i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;

    // MPU power on
    PIN_setOutputValue(hMpuPin, Board_MPU_POWER, Board_MPU_POWER_ON);

    // MPU open i2c
    i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
    if (i2cMPU == NULL) {
        System_abort("Error Initializing I2CMPU\n");
    }

    // MPU setup
    mpu9250_setup(&i2cMPU);

    while(1) {

        if (index < 20 && programState == READ_SENSOR) {
            // Read data
            mpu9250_get_data(&i2cMPU, &ax, &ay, &az, &gx, &gy, &gz);
            //store values into list
            arr_ax[index] = ax;
            arr_gz[index] = gz;

            if (index == 19) {
                //store movement return value in variable
                int move = movement(arr_ax, arr_gz, index + 1);

                // Check if right movement is done
                if(move == 1){
                    // Fast pull, direction up
                    programState = UPDATE;

                } else if (move == 2) {
                    // Too slow pull, direction up
                    buzzerSound(2);

                } else if (move == 3) {
                    // Laying
                    programState = SLEEP;
                    marioTheme();
                }
            }
            index++;

        } else {
            index = 0; //reset index
        }
        Task_sleep(100000 / Clock_tickPeriod); // Puts Task to sleep (100ms)

    }
    // Program is never closed
    // MPU close i2c
    // I2C_close(i2cMPU);
    // MPU power off
    // PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_OFF);
}

void uartFxn(UART_Handle uart, void *rxBuffer, size_t len) {

    // Checks if uartBuffer has substring "3033,BEEP" in it
    if ((strstr((char*)rxBuffer, "3033,BEEP:Too late") != NULL)) {

        // Change state to RUNAWAY
        programState = RUNAWAY;

    } else if ((strstr((char*)rxBuffer, "3033,BEEP") != NULL)) {

        // Change state to NEW_MSG
        programState = NEW_MSG;

    }
    UART_read(uart, rxBuffer, len);
}

void uartTask(UArg arg0, UArg arg1) {

    //Variables
    uint8_t uartBuffer[80];
    char str[22];

    UART_Handle uart;
    UART_Params uartParams;

    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.readMode = UART_MODE_CALLBACK;
    uartParams.readCallback = &uartFxn; // Handler function
    uartParams.baudRate = 9600; // Speed 9600baud
    uartParams.dataLength = UART_LEN_8; // 8
    uartParams.parityType = UART_PAR_NONE; // n
    uartParams.stopBits = UART_STOP_ONE; // 1

    uart = UART_open(Board_UART0, &uartParams);
    if (uart == NULL) {
       System_abort("Error opening the UART");
    }

    UART_read(uart, uartBuffer, (sizeof(uartBuffer)));

    while (1) {

        switch(programState) {
            case NEW_MSG:
                Task_sleep(1000000 / Clock_tickPeriod); // Wait second
                buzzerSound(3);
                programState = READ_SENSOR;
                break;

            case RUNAWAY:
                Task_sleep(1000000 / Clock_tickPeriod); // Wait second
                buzzerSound(4);
                programState = READ_SENSOR;
                break;

            case UPDATE:
                buzzerSound(1);
                str[0] = '\0';
                if (activityState == ACTIVATE) {
                    sprintf(str, "id:3033,ACTIVATE:1;1;1\0");
                    UART_write(uart, str, (strlen(str) + 1));
                } else if (activityState == EAT) {
                    sprintf(str, "id:3033,EAT:2\0");
                    UART_write(uart, str, (strlen(str) + 1));
                } else if (activityState == EXERCISE) {
                    sprintf(str, "id:3033,EXERCISE:2\0");
                    UART_write(uart, str, (strlen(str) + 1));
                } else if (activityState == PET) {
                    sprintf(str, "id:3033,PET:2\0");
                    UART_write(uart, str, (strlen(str) + 1));
                }

                programState = READ_SENSOR;
        }

        Task_sleep(200000 / Clock_tickPeriod);
    }
}

Int main(void) {

    // Task variables
    Task_Handle mpuTaskHandle;
    Task_Params mpuTaskParams;
    Task_Handle uartTaskHandle;
    Task_Params uartTaskParams;

    // Initialize Board and i2c bus
    Board_initGeneral();
    Board_initI2C();
    Board_initUART();

    // Open MPU power pin
    hMpuPin = PIN_open(&MpuPinState, MpuPinConfig);
    if (hMpuPin == NULL) {
        System_abort("Pin open failed!");
    }

    /* Task */
    Task_Params_init(&mpuTaskParams);
    mpuTaskParams.stackSize = STACKSIZE;
    mpuTaskParams.stack = &mpuTaskStack;
    mpuTaskParams.priority = 2;
    mpuTaskHandle = Task_create(mpuTask, &mpuTaskParams, NULL);
    if(mpuTaskHandle == NULL) {
        System_abort("Task create failed");
    }

    Task_Params_init(&uartTaskParams);
    uartTaskParams.stackSize = STACKSIZE;
    uartTaskParams.stack = &uartTaskStack;
    uartTaskParams.priority = 2;
    uartTaskHandle = Task_create(uartTask, &uartTaskParams, NULL);
    if (uartTaskHandle == NULL) {
        System_abort("Task create failed!");
    }

    // Initialize button pins
    buttonHandle = PIN_open(&buttonState, buttonConfig);
    if(!buttonHandle) {
       System_abort("Error initializing button pins\n");
    }

    // Initialize button pins
    ledHandle = PIN_open(&ledState, ledConfig);
    if(!ledHandle) {
       System_abort("Error initializing LED pins\n");
    }

    // Initialize buzzer
    hBuzzer = PIN_open(&sBuzzer, cBuzzer);
    if (hBuzzer == NULL) {
        System_abort("Pin open failed!");
    }

    // Set the button pin's interrupt handler to the function buttonFxn
    if (PIN_registerIntCb(buttonHandle, &buttonFxn) != 0) {
       System_abort("Error registering button callback function");
    }

    /* Send hello to console */
    System_printf("Hello world!\n");
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
