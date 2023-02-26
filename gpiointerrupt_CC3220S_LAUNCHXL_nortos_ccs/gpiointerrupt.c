/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/Timer.h>

/* Driver configuration */
#include "ti_drivers_config.h"

// for UART driver
#define DISPLAY(x) UART_write(uart, &output, x);

// UART global variables
char output[64];
int bytesToSend;

// driver handles global variables
UART_Handle uart;

void initUART(void) {
    UART_Params uartParams;

    // Init the driver
    UART_init();

    // configure driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    // open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);
    if (uart = NULL) {

        // uart open() failed
        while (1);
    }

}

// for I2C driver

// I2C Global Variables
static const struct {
        uint8_t address;
        uint8_t resultReg;
        char *id;
    } sensors[3] = {
        { 0x48, 0x0000, "11X" },
        { 0x49, 0x0000, "116" },
        { 0x41, 0x0001, "006" }
};

uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;


// Driver Handles - Global variables
I2C_Handle i2c;

// call initUART() before calling this function
void initI2C(void) {

    int8_t i, found;
    I2C_Params i2cParams;

    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))

    // init the driver
    I2C_init();

    // config the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL) {
        DISPLAY(snprintf(output, 64, "Failed\n\r"))
        while (1);
    }
    DISPLAY(snprintf(output, 32, "Passed\n\r"))


    // common I2C transaction setup
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    found = false;
    for (i=0; i<3; ++i) {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))

        if (I2C_transfer(i2c, &i2cTransaction)) {

            DISPLAY(snprintf(output, 64, "Found\n\r"))
            found = true;
            break;
        }
            DISPLAY(snprintf(output, 64, "No\n\r"))
        }

    if(found) {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.slaveAddress))
    }

    else {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"))
    }


}

int16_t readTemp(void) {
    int j;
    int16_t temperature = 0;

    i2cTransaction.readCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction)) {

                /*
                * Extract degrees C from the received data;
                * see TMP sensor datasheet
                */
                temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
                temperature *= 0.0078125;
                /*
                * If the MSB is set '1', then we have a 2's complement
                * negative value which needs to be sign extended
                */
                if (rxBuffer[0] & 0x80)
                {
                temperature |= 0xF000;
                }

    }

    else {

        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r",i2cTransaction.status))
        DISPLAY(snprintf(output, 64, "Please power cycle your board by nplugging USB and plugging back in.\n\r"))

    }

    return temperature;

}

// for timer driver

// Driver handles - global variables
Timer_Handle timer0;

volatile unsigned char TimerFlag = 0;

void timerCallback(Timer_Handle myHandle, int_fast16_t status) {

    TimerFlag = 1;
}

void initTimer(void) {
    Timer_Params params;

            // Init the driver
            Timer_init();

            // Configure the driver
            Timer_Params_init(&params);
            params.period = 1000000;
            params.periodUnits = Timer_PERIOD_US;
            params.timerMode = Timer_CONTINUOUS_CALLBACK;
            params.timerCallback = timerCallback;

            // Open the driver
            timer0 = Timer_open(CONFIG_TIMER_0, &params);

            if (timer0 == NULL) {
                /* Failed to initialized timer */
                while (1) {}
            }

            if (Timer_start(timer0) == Timer_STATUS_ERROR) {
                /* Failed to start timer */
                while (1) {}
            }

}

// state

// setting buttonClick
unsigned char Bttn0_Click;
unsigned char Bttn1_Click;

// thermostat variables
int setpoint;
int temperature;
int heat;


// temperature state
enum TEMP_STATES {
    TEMP_Start,
    TEMP_Raise,
    TEMP_Adjust,
} TEMP_STATE;


void Bttn_Click() {
    // switch statement for temp state
    switch(TEMP_STATE) {
        case TEMP_Start:
            // buttons not clicked
            Bttn0_Click = 0;
            Bttn1_Click = 1;
            TEMP_STATE = TEMP_Raise;
            break;

        case TEMP_Raise:
            // if button clicked
            if (Bttn0_Click || Bttn1_Click) {
                TEMP_STATE = TEMP_Raise;
            }
            break;

        case TEMP_Adjust:
            // if button not clicked
            if (!(Bttn0_Click || Bttn1_Click)) {
                TEMP_STATE = TEMP_Raise;
            }
            break;

        default:
            TEMP_STATE = TEMP_Start;
            break;

    }

    // state temp actions
    switch(TEMP_STATE) {
        case TEMP_Start:
            break;

        case TEMP_Raise:
            break;

        case TEMP_Adjust:
            if (Bttn0_Click) {
                setpoint -=1;
                Bttn0_Click = 0;
            }
            if (Bttn1_Click) {
                setpoint -=1;
                Bttn1_Click = 0;
            }
            break;

        default:
            break;
    }
}


// LEDs
enum LED_STATES {
    LED_Start,
    LED_On,
    LED_Off,
} LED_STATE;

void LED_tempChange() {
    switch(LED_STATE) {
        case LED_Start:
            heat = 0;
            LED_STATE = LED_Off;
            break;

        case LED_On:
            if (temperature > setpoint) {
                LED_STATE = LED_Off;
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                heat = 0;
            }
            break;

        case LED_Off:
            if (temperature < setpoint) {
                LED_STATE = LED_On;
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
                heat = 1;
            }
            break;
        default:
            LED_STATE = LED_Start;
            break;
    }

    switch (LED_STATE) {
        case LED_Start:
            break;
        case LED_On:
            break;
        case LED_Off:
            break;
        default:
            break;
    }
}




/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    /* Toggle an LED */
    GPIO_toggle(CONFIG_GPIO_LED_0);
    Bttn1_Click = 1;
}


/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    initUART();
    initI2C();
    initTimer();
    GPIO_init();

    int seconds = 0;
    const unsigned int timerPeriod = 100000;
    unsigned int bttnTimer = 200000;
    unsigned int tempCheck = 500000;
    unsigned int uartTimer = 100000;

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);

    temperature = readTemp();
    setpoint = readTemp();

    // setting state machine
    TEMP_STATE = TEMP_Start;
    LED_STATE = LED_Start;

    while (1) {
        if (bttnTimer > 200000) {
            Bttn_Click();
            bttnTimer = 0;
        }
        if (tempCheck >= 500000) {
            temperature = readTemp();
            LED_tempChange();
            tempCheck = 0;
        }
        if (uartTimer >= 100000) {
            DISPLAY(snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", temperature, setpoint, heat, seconds))
            seconds += 1;
            uartTimer = 0;
        }
        while(!TimerFlag) {
        }
            TimerFlag = 0;
            bttnTimer += timerPeriod;
            tempCheck += timerPeriod;
            uartTimer += timerPeriod;
    }
}
