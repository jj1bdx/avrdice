/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Kenji Rikitake
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/* #define F_CPU (16000000UL) */

// Test code for avrhwrng v2rev1 with
// Adafruit I2C LCD unit + DFR0090 SPI 8-digit LED module

// For DFR0090: using internal SPI unit
// With Pin D10(SS): Latch, D11(MOSI): Data, D13(SCK): Clock

// NOTE WELL:
// * Arduino + LCD consumes ~110mA
// * Arduino + LCD + LED consumes ~320mA
// * Power supply from USB (5V) will work
// * Power supply from DC jack of 7.5V will work (may generate heat)
// * Power supply from DC jack of 12V will NOT work (too much heat)

/* 
 * Show avrhwrng output as 1 <= X <= 6 value on the top line
 * The bottom line shows history of the past 14 attempts
 */

// AVRTools headers

#include "AVRTools/ArduinoPins.h"
#include "AVRTools/InitSystem.h"
#include "AVRTools/SystemClock.h"
#include "AVRTools/I2cMaster.h"
#include "AVRTools/I2cLcd.h"
#include "AVRTools/SPI.h"
#include "AVRTools/USART0.h"

// Digit to LED pattern conversion table

uint8_t LEDtable[] =
    {0xc0, 0xf9, 0xa4, 0xb0, 0x99,
     0x92, 0x82, 0xf8, 0x80, 0x90,
     0xff};

// Arduino UNO SPI pins

#define pLatch pSS   // pPin10
#define pData  pMOSI // pPin11
#define pClock pSCK  // pPin13

// Send a character to LED via SPI device
// This will shift all the letters
// from left to right

void LEDsend(uint8_t data) {

    setGpioPinLow(pSS);
    (void)SPI::transmit(data);
    setGpioPinHigh(pSS);
}

// Generate dice values of 1 to 6
// from 8-bit sampled data of avrhwrng
// screened with the von Neumann algorithm
// for each bit of avrhwrng

uint8_t rngdice() {

    uint8_t s, v0, v1, outval, count;
    bool notok;

    notok = true;
    s = 0;
    count = 0;
    outval = 0;

    // Loop until sufficent bits are gathered
    while (notok) {
        // sample two bits and do the von Neumann test
        if (s == 0) {
            // 1st bit state
            v0 = readGpioPinDigitalV(makeGpioVarFromGpioPin(pPin06));
            v1 = readGpioPinDigitalV(makeGpioVarFromGpioPin(pPin07));
            s = 1;
        } else {
            // perform von Neumann test for each bit
            if (v0 != readGpioPinDigitalV(makeGpioVarFromGpioPin(pPin06))) {
                outval = outval + outval;
                if (v0 == 1) {
                    outval++;
                }
                count++;
            }
            if (v1 != readGpioPinDigitalV(makeGpioVarFromGpioPin(pPin07))) {
                outval = outval + outval;
                if (v1 == 1) {
                    outval++;
                }
                count++;
            }
            s = 0;
        }
        // 0 <= outval <= 255
        if (count >= 8) {
            count = 0;
            // 252/6 = 42
            // Discard unmappable values
            if (outval < 252) {
                // Exit the loop when outval is within the range
                notok = false;
            }
        }
    }
    // Get the dice value from outval
    return (outval % 6) + 1;
}

// main program

int main() {
    I2cLcd lcd;
    uint8_t d;
    uint8_t p[15];
    uint8_t i;
    uint8_t c;

    // Initialize
    initSystem();
    initSystemClock();
    I2cMaster::start();
    USART0::start(9600); // 9600bps for serial
    SPI::enable();
    // Max 4MHz for slave, 8MHz for master
    SPI::configure(SPI::SPISettings(8000000,
                SPI::kMsbFirst, SPI::kSpiMode0));
    setGpioPinHigh(pSS);
    // HRNG bits of avrhwrng
    setGpioPinModeInput(pPin07);
    setGpioPinModeInput(pPin06);
    // clean up the LED buffer
    for (i = 0; i < sizeof(p); i++) {
        p[i] = 0x30;
    }
    // initialize the LCD
    lcd.init();
    lcd.clear();
    lcd.home();
    lcd.autoscrollOff();
    lcd.setBacklight(I2cLcd::kBacklight_White);

    // Main loop
    for(;;) {
        // Gather the HRNG bits and the random value
        d = rngdice();
        // Show the value to the LCD
        lcd.setCursor(0, 0);
        lcd.print("Dice = ");
        lcd.print((int)d);
        lcd.setCursor(1, 0);
        // Shift the LCD output buffer
        for (i = 0; i < sizeof(p) - 1; i++) {
            p[i] = p[i + 1];
        }
        // Put the new dice value to the LCD
        c = 0x30 + d;
        p[sizeof(p) - 1] = c;
        // Output the new dice value to the serial port
        USART0::write((char)c);
        // Refresh the LCD output
        for (i = 0; i < sizeof(p); i++) {
            lcd.print((char)p[i]);
        }
        // Refresh the LED output
        for (i = sizeof(p) - 1; i >= (sizeof(p) - 8); i--) {
            LEDsend(LEDtable[p[i]-0x30]);
        }
        // Wait for one second
        delay(1000);
    }
}
