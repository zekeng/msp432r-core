/*
 * Copyright (c) 2015, Texas Instruments Incorporated
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

#include <ti/runtime/wiring/wiring_private.h>
#include <ti/runtime/wiring/msp432/wiring_analog.h>

#include <driverlib/adc14.h>

#include <ti/drivers/PWM.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOMSP432.h>

uint8_t digital_pin_to_pin_function[] = {
    PIN_FUNC_UNUSED,    /*  dummy */

    PIN_FUNC_UNUSED,    /*  1 - BUTTON1 - User Button 1 */
    PIN_FUNC_UNUSED,    /*  2 - BUTTON2 - User Button 2 */
    PIN_FUNC_UNUSED,    /*  3 - BCLUART_RXD - Debug Serial RX */
    PIN_FUNC_UNUSED,    /*  4 - BCLUART_TXD - Debug Serial TX */
    PIN_FUNC_UNUSED,    /*  5 - FL_SPI_CS# - FLASH Chip Select (active low) */
    PIN_FUNC_UNUSED,    /*  6 - FL_SPI_CLK - FLASH SPI CLOCK */
    PIN_FUNC_UNUSED,    /*  7 - FL_SPI_MOSI - FLASH Master Output Slave Input */
    PIN_FUNC_UNUSED,    /*  8 - FL_SPI_MISO - FLASH Master Input Slave Output */
    PIN_FUNC_UNUSED,    /*  9 - SIM800_UART_CTS - SIM800 UART CTS */
    PIN_FUNC_UNUSED,    /* 10 - SIM800_UART_RTS - SIM800 UART RTS */
    PIN_FUNC_UNUSED,    /* 11 - SIM800_UART_RX - SIM800 UART RX */
    PIN_FUNC_UNUSED,    /* 12 - SIM800_UART_TX - SIM800 UART TX */
    PIN_FUNC_UNUSED,    /* 13 - A1_PD_SCK - SENSOR A1 CLOCK */
    PIN_FUNC_UNUSED,    /* 14 - B1_PD_SCK - SENSOR B1 CLOCK */
    PIN_FUNC_UNUSED,    /* 15 - BT_RXD - Bluetooth UART RX */
    PIN_FUNC_UNUSED,    /* 16 - BT_TXD - Bluetooth UART TX */
    PIN_FUNC_UNUSED,    /* 17 - GSM_PWR_EN - GSM Power Enable */
    PIN_FUNC_UNUSED,    /* 18 - SIM800_PWR_ON - SIM800 Power On */
    PIN_FUNC_UNUSED,    /* 19 - BT_GPIO1 - Bluetooth GPIO1 */
    PIN_FUNC_UNUSED,    /* 20 - BT_GPIO2 - Bluetooth GPIO2 */
    PIN_FUNC_UNUSED,    /* 21 - SIM800_RING - SIM800 RING */
    PIN_FUNC_UNUSED,    /* 22 - SIM800_NET_STAT - SIM800 NET STATUS */
    PIN_FUNC_UNUSED,    /* 23 - SIM800_STAT - SIM800 STATUS */
    PIN_FUNC_UNUSED,    /* 24 - SIM800_DCD - SIM800 DCD */
    PIN_FUNC_UNUSED,    /* 25 - SIM800_PWR_MON - SIM800 Power Monitor */
    PIN_FUNC_UNUSED,    /* 26 - A2_PD_SCK - Sensor A2 CLOCK */
    PIN_FUNC_UNUSED,    /* 27 - A1_DOUT - Sensor A1 Digital OUTPUT */
    PIN_FUNC_UNUSED,    /* 28 - B1_DOUT - Sensor B1 Digital OUTPUT */
    PIN_FUNC_UNUSED,    /* 29 - A2_DOUT - Sensor A2 Digital OUTPUT */
    PIN_FUNC_UNUSED,    /* 30 - B2_DOUT - Sensor B2 Digital OUTPUT */
    PIN_FUNC_UNUSED,    /* 31 - TMP_ALERT - TMP1075 Alert Pin */
    PIN_FUNC_UNUSED,    /* 32 - Vbat_MSRMNT - Vbatt Measurement Signal */
    PIN_FUNC_UNUSED,    /* 33 - EN3V3 - 3.3V Enable Pin */
    PIN_FUNC_UNUSED,    /* 34 - EN_Vbat_SW - Vbatt Enable */
    PIN_FUNC_UNUSED,    /* 35 - I2C_SDA - TMP1075 SDA */
    PIN_FUNC_UNUSED,    /* 36 - I2C_SCL - TMP1075 SCL */
    PIN_FUNC_UNUSED,    /* 37 - DQ1 DS18B20 - Data Pin (not used) */
    PIN_FUNC_UNUSED,    /* 38 - DQ2 DS18B20 - Data Pin (not used) */
    PIN_FUNC_UNUSED,    /* 39 - LED_RED - Red LED */
    PIN_FUNC_UNUSED,    /* 40 - LED_GREEN - Green LED */
    PIN_FUNC_UNUSED,    /* 41 - CLEAR_WDT - External WDT Clear Pin */
    PIN_FUNC_UNUSED     /* 42 - B2_PD_SCK - SENSOR B2 CLOCK */
};

/*
 * When a mappable pin is being used for analogWrite(),
 * its corresponding entry in this table is replaced with the
 * PWM channel index it is using.
 *
 * If/when a pin is then changed back to a digitial pin, the
 * pin's entry in this table is restored to PWM_MAPPABLE.
 *
 * Fixed map entries are not modified.
 */
uint8_t digital_pin_to_pwm_index[] = {
    PWM_NOT_MAPPABLE,       /*  dummy */

    PWM_NOT_MAPPABLE,    /*  1 - BUTTON1 - User Button 1 */
    PWM_NOT_MAPPABLE,    /*  2 - BUTTON2 - User Button 2 */
    PWM_NOT_MAPPABLE,    /*  3 - BCLUART_RXD - Debug Serial RX */
    PWM_NOT_MAPPABLE,    /*  4 - BCLUART_TXD - Debug Serial TX */
    PWM_NOT_MAPPABLE,    /*  5 - FL_SPI_CS# - FLASH Chip Select (active low) */
    PWM_NOT_MAPPABLE,    /*  6 - FL_SPI_CLK - FLASH SPI CLOCK */
    PWM_NOT_MAPPABLE,    /*  7 - FL_SPI_MOSI - FLASH Master Output Slave Input */
    PWM_NOT_MAPPABLE,    /*  8 - FL_SPI_MISO - FLASH Master Input Slave Output */
    PWM_NOT_MAPPABLE,    /*  9 - SIM800_UART_CTS - SIM800 UART CTS */
    PWM_NOT_MAPPABLE,    /* 10 - SIM800_UART_RTS - SIM800 UART RTS */
    PWM_NOT_MAPPABLE,    /* 11 - SIM800_UART_RX - SIM800 UART RX */
    PWM_NOT_MAPPABLE,    /* 12 - SIM800_UART_TX - SIM800 UART TX */
    PWM_NOT_MAPPABLE,    /* 13 - A1_PD_SCK - SENSOR A1 CLOCK */
    PWM_NOT_MAPPABLE,    /* 14 - B1_PD_SCK - SENSOR B1 CLOCK */
    PWM_NOT_MAPPABLE,    /* 15 - BT_RXD - Bluetooth UART RX */
    PWM_NOT_MAPPABLE,    /* 16 - BT_TXD - Bluetooth UART TX */
    PWM_NOT_MAPPABLE,    /* 17 - GSM_PWR_EN - GSM Power Enable */
    PWM_NOT_MAPPABLE,    /* 18 - SIM800_PWR_ON - SIM800 Power On */
    PWM_NOT_MAPPABLE,    /* 19 - BT_GPIO1 - Bluetooth GPIO1 */
    PWM_NOT_MAPPABLE,    /* 20 - BT_GPIO2 - Bluetooth GPIO2 */
    PWM_NOT_MAPPABLE,    /* 21 - SIM800_RING - SIM800 RING */
    PWM_NOT_MAPPABLE,    /* 22 - SIM800_NET_STAT - SIM800 NET STATUS */
    PWM_NOT_MAPPABLE,    /* 23 - SIM800_STAT - SIM800 STATUS */
    PWM_NOT_MAPPABLE,    /* 24 - SIM800_DCD - SIM800 DCD */
    PWM_NOT_MAPPABLE,    /* 25 - SIM800_PWR_MON - SIM800 Power Monitor */
    PWM_NOT_MAPPABLE,    /* 26 - A2_PD_SCK - Sensor A2 CLOCK */
    PWM_NOT_MAPPABLE,    /* 27 - A1_DOUT - Sensor A1 Digital OUTPUT */
    PWM_NOT_MAPPABLE,    /* 28 - B1_DOUT - Sensor B1 Digital OUTPUT */
    PWM_NOT_MAPPABLE,    /* 29 - A2_DOUT - Sensor A2 Digital OUTPUT */
    PWM_NOT_MAPPABLE,    /* 30 - B2_DOUT - Sensor B2 Digital OUTPUT */
    PWM_NOT_MAPPABLE,    /* 31 - TMP_ALERT - TMP1075 Alert Pin */
    PWM_NOT_MAPPABLE,    /* 32 - Vbat_MSRMNT - Vbatt Measurement Signal */
    PWM_NOT_MAPPABLE,    /* 33 - EN3V3 - 3.3V Enable Pin */
    PWM_NOT_MAPPABLE,    /* 34 - EN_Vbat_SW - Vbatt Enable */
    PWM_NOT_MAPPABLE,    /* 35 - I2C_SDA - TMP1075 SDA */
    PWM_NOT_MAPPABLE,    /* 36 - I2C_SCL - TMP1075 SCL */
    PWM_NOT_MAPPABLE,    /* 37 - DQ1 DS18B20 - Data Pin (not used) */
    PWM_NOT_MAPPABLE,    /* 38 - DQ2 DS18B20 - Data Pin (not used) */
    PWM_NOT_MAPPABLE,    /* 39 - LED_RED - Red LED */
    PWM_NOT_MAPPABLE,    /* 40 - LED_GREEN - Green LED */
    PWM_NOT_MAPPABLE,    /* 41 - CLEAR_WDT - External WDT Clear Pin */
    PWM_NOT_MAPPABLE     /* 42 - B2_PD_SCK - SENSOR B2 CLOCK */
};

/*
 * mapping of pins to an ADC channel
 */
const uint8_t digital_pin_to_adc_index[] = {
    /* port_pin */
    NOT_ON_ADC,     /*  dummy */

    NOT_ON_ADC,    /*  1 - BUTTON1 - User Button 1 */
    NOT_ON_ADC,    /*  2 - BUTTON2 - User Button 2 */
    NOT_ON_ADC,    /*  3 - BCLUART_RXD - Debug Serial RX */
    NOT_ON_ADC,    /*  4 - BCLUART_TXD - Debug Serial TX */
    NOT_ON_ADC,    /*  5 - FL_SPI_CS# - FLASH Chip Select (active low) */
    NOT_ON_ADC,    /*  6 - FL_SPI_CLK - FLASH SPI CLOCK */
    NOT_ON_ADC,    /*  7 - FL_SPI_MOSI - FLASH Master Output Slave Input */
    NOT_ON_ADC,    /*  8 - FL_SPI_MISO - FLASH Master Input Slave Output */
    NOT_ON_ADC,    /*  9 - SIM800_UART_CTS - SIM800 UART CTS */
    NOT_ON_ADC,    /* 10 - SIM800_UART_RTS - SIM800 UART RTS */
    NOT_ON_ADC,    /* 11 - SIM800_UART_RX - SIM800 UART RX */
    NOT_ON_ADC,    /* 12 - SIM800_UART_TX - SIM800 UART TX */
    NOT_ON_ADC,    /* 13 - A1_PD_SCK - SENSOR A1 CLOCK */
    NOT_ON_ADC,    /* 14 - B1_PD_SCK - SENSOR B1 CLOCK */
    NOT_ON_ADC,    /* 15 - BT_RXD - Bluetooth UART RX */
    NOT_ON_ADC,    /* 16 - BT_TXD - Bluetooth UART TX */
    NOT_ON_ADC,    /* 17 - GSM_PWR_EN - GSM Power Enable */
    NOT_ON_ADC,    /* 18 - SIM800_PWR_ON - SIM800 Power On */
    NOT_ON_ADC,    /* 19 - BT_GPIO1 - Bluetooth GPIO1 */
    NOT_ON_ADC,    /* 20 - BT_GPIO2 - Bluetooth GPIO2 */
    NOT_ON_ADC,    /* 21 - SIM800_RING - SIM800 RING */
    NOT_ON_ADC,    /* 22 - SIM800_NET_STAT - SIM800 NET STATUS */
    NOT_ON_ADC,    /* 23 - SIM800_STAT - SIM800 STATUS */
    NOT_ON_ADC,    /* 24 - SIM800_DCD - SIM800 DCD */
    NOT_ON_ADC,    /* 25 - SIM800_PWR_MON - SIM800 Power Monitor */
    NOT_ON_ADC,    /* 26 - A2_PD_SCK - Sensor A2 CLOCK */
    NOT_ON_ADC,    /* 27 - A1_DOUT - Sensor A1 Digital OUTPUT */
    NOT_ON_ADC,    /* 28 - B1_DOUT - Sensor B1 Digital OUTPUT */
    NOT_ON_ADC,    /* 29 - A2_DOUT - Sensor A2 Digital OUTPUT */
    NOT_ON_ADC,    /* 30 - B2_DOUT - Sensor B2 Digital OUTPUT */
    NOT_ON_ADC,    /* 31 - TMP_ALERT - TMP1075 Alert Pin */
             0,    /* 32 - Vbat_MSRMNT - Vbatt Measurement Signal */
    NOT_ON_ADC,    /* 33 - EN3V3 - 3.3V Enable Pin */
    NOT_ON_ADC,    /* 34 - EN_Vbat_SW - Vbatt Enable */
    NOT_ON_ADC,    /* 35 - I2C_SDA - TMP1075 SDA */
    NOT_ON_ADC,    /* 36 - I2C_SCL - TMP1075 SCL */
    NOT_ON_ADC,    /* 37 - DQ1 DS18B20 - Data Pin (not used) */
    NOT_ON_ADC,    /* 38 - DQ2 DS18B20 - Data Pin (not used) */
    NOT_ON_ADC,    /* 39 - LED_RED - Red LED */
    NOT_ON_ADC,    /* 40 - LED_GREEN - Green LED */
    NOT_ON_ADC,    /* 41 - CLEAR_WDT - External WDT Clear Pin */
    NOT_ON_ADC     /* 42 - B2_PD_SCK - SENSOR B2 CLOCK */
};

