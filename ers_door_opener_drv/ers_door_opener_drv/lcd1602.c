/*
 * lcd1602.c
 *
 */

#include "lcd1602.h"
#include <stdbool.h>
#include <driverlib.h>

#define RS 0x01
#define RW 0x02
#define EN 0x04

#define BACKLIGHT_ON 0x08
#define BACKLIGHT_OFF 0x00

volatile static uint8_t backlight_state = 0x00;


void write8BitI2CtoDisplay(uint8_t data);
void write4BitI2CtoDisplay(uint8_t data, bool cmd);
extern void sleep(uint16_t ms);


lcd1602_res_t lcd1602_init(void) {
    //mindestens 40ms nach Power On warten, bis mit der Initialisierung des Displays begonnen wird:
    sleep(40);

    //Step1 - Function set: 0 0 1 DL N F - -
    //DataLength (DL) = 1 --> 8 Bit
    //Num od Displ. Lines (NL) und Char Font (F) sind ndef, da mit diesem ersten 8-Bit Befehl
    //nur D7-D4 �bertragen werden k�nnen (D3-D0 sind ja nicht angeschlossen)
    //Data: 0011 0000
    //CtrlNib = 0000
    //nach Befehl mindestens 4,1 ms warten
    write8BitI2CtoDisplay(0X30);
    sleep(10);

    //Step2 - Wie Step 1; mindestens 100�s warten
    write8BitI2CtoDisplay(0X30);
    sleep(1);

    //Step3 - wie Step 1
    write8BitI2CtoDisplay(0X30);
    sleep(1);

    //Step4 - Function Set: 0 0 1 DL N F - -
    //Display nun auf 4 Bit Mode einstellen; dieser Befehl wird noch als
    //8-Bit Befehl verstanden --> nur ein Aufruf; das lowLCD - Nibble ist wiederum nicht definiert
    //Daher muss der Befehl anschlie�end im 4-Bit Mode nochmals wiederholt werden.
    //DataLength (DL) = 0 --> 4 Bit
    //Data: 0010 0000
    //CtrlNib = 0000
    write8BitI2CtoDisplay(0X20);
    sleep(1);

    //Step5 - Function Set: 0 0 1 DL N F - -
    //Nun Einstellung der Anzahl der Zeilen und der Font Matrix
    //DataLength (DL) = 0 --> 4 Bit
    //Num. of Lines (N) = 1 --> 2 Zeilen
    //Char Font (F) = 0 --> 5x8 Dot Matrix
    //Data: 0010 1000
    write4BitI2CtoDisplay(0x28, true);
    sleep(1);

    //Step6 - Display on/off: 0000 1 D C B
    //Display ausschalten
    //Display on/off (D) = 0 --> off
    //Cursor on/off (C) = 0 --> off
    //Blinking on/off (B) = 0 --> off
    //Data: 0000 1000
    write4BitI2CtoDisplay(0x08, true);
    sleep(1);

    //Step7 - Display Clear: 0000 0001
    write4BitI2CtoDisplay(0x01, true);
    sleep(5);

    //Step8 - Entry mode set: 0000 01 I/D S
    //Increment/Decrement (I/D) = 1 --> Inc
    //Display Shift (S) = 0 --> no shift
    //Data: 0000 0110
    write4BitI2CtoDisplay(0x06, true);
    sleep(1);

    //Step9 - Display On/Off: 0000 1 D C B
    //Display on/off (D) = 1 --> on
    //Cursor on/off (C) = 0 --> off
    //Blinking on/off (B) = 0 --> off
    //Data: 0000 1100
    write4BitI2CtoDisplay(0x0C, true);
    sleep(1);
    return eLCD1602_ok;
}

lcd1602_res_t lcd1602_write(uint16_t line, char* text) {
    char tmpChar;
    char cmd;
    uint16_t ii;

    // Cursor platzieren
    if (line == 1) {
        //return home cmd: 0000 001x
        cmd = 0x02;
    } else if (line == 2) {
        //Set DDRAM address: 1 xxx xxxx
        //Cursor auf den Anfang der zweiten Zeile stellen
        //Die zweite Zeile startet im DDRAM  bei Adresse 0x40
        //Data: 1000 0000
        // xxx xxxx = 0x40 = 100 0000
        //--> cmd = 1100 0000
        cmd = 0xC0;
    } else
        return eLCD1602_invalidLine;

    write4BitI2CtoDisplay(cmd, true);
    sleep(3);

    // Text �bertragen
    ii= 0;
    tmpChar = *text;
    if (tmpChar == 0)
        return eLCD1602_ok;

    while (tmpChar != 0) {
        write4BitI2CtoDisplay(tmpChar, false);
        tmpChar = *(++text);
        if (++ii>=16) {
            break;
        }
    }
    return eLCD1602_ok;
}


lcd1602_res_t lcd1602_clear(void) {
    //Display Clear: 0000 0001
    write4BitI2CtoDisplay(0x01, true);
    sleep(5);
    return eLCD1602_ok;
}

lcd1602_res_t lcd1602_backlight(bool on) {
    if (on) {
        backlight_state = BACKLIGHT_ON;
    } else {
        backlight_state = BACKLIGHT_OFF;
    }
    EUSCI_B_I2C_masterSendSingleByte(EUSCI_B0_BASE, backlight_state);
    while (EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
    sleep(1);
    return eLCD1602_ok;
}

bool lcd1602_getBacklightState(void) {
    return backlight_state;
}



void write8BitI2CtoDisplay(uint8_t data) {
    data = data|backlight_state;
    EUSCI_B_I2C_masterSendSingleByte(EUSCI_B0_BASE, data);
    while (EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
    sleep(1);
    EUSCI_B_I2C_masterSendSingleByte(EUSCI_B0_BASE, data|EN);
    while (EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
    sleep(1);
    EUSCI_B_I2C_masterSendSingleByte(EUSCI_B0_BASE, data);
    while (EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
    sleep(1);
}

void write4BitI2CtoDisplay(uint8_t data, bool cmd) {
    uint16_t mask = backlight_state;
    uint8_t local_data;

    if (!cmd)
        mask |= 0x01;

    local_data = (data & 0xF0)|mask;
    write8BitI2CtoDisplay(local_data);

    local_data = ((data<<4)& 0xF0)|mask;
    write8BitI2CtoDisplay(local_data);
}
