/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
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
 * --/COPYRIGHT--*/
//*****************************************************************************
// This example shows how to configure the I2C module as a master for
// single byte transmission in interrupt mode. The address of the slave
// module that the master is communicating with also set in this example.
//
//  Demo - EUSCI_B0 I2C Master TX single bytes to MSP430 Slave
//  Description: This demo connects two MSP430's via the I2C bus. The master
//  transmits to the slave. This is the master code. It continuously
//  transmits 00h, 01h, ..., 0ffh and demonstrates how to implement an I2C
//  master transmitter sending a single byte using the USCI_B0 TX interrupt.
//  ACLK = n/a, MCLK = SMCLK = BRCLK = default DCO = ~1MHz
//
//  TI sample project eusci_b_i2c_ex4_masterTxSingle
//  Modified by S. Steddin
//  2020-07-14
//******************************************************************************
/*
 * Allgemeine Zweckbestimmung:
 * Demonstration, wie das LCD 1602 in ein MSP430FR2355 Projekt eingebunden werden kann
 *
 * Version 0.4 (erneut validiert am 06.05.2024)
 * Zweckbestimmung
 * ZB-04-001: Ausgeben von Text auf dem Display
 *
 * Bestimmungsgem��er Gebrauch
 * 1. �bersetzen des Projektes
 * 2. Starten des Projekts im Debug mode
 * 3. Kontrolle der Textanzeige
 *
 * Funktionale Anforderungen
 * FA-001: rote LED soll timergesteuert blinken
 * FA-002: Hintergrundbeleuchtung des LCD1602 soll an- und ausgeschaltet werden
 * FA-003: Vordefinierte Textstrings sollen ausgegeben werden
 *
 * Nichtfunktionale Anforderungen
 * NFA-001: Timer soll Interruptroutine implementieren, �ber welche die LED angesteuert wird
 * NFA-002: API wird in Modul lcd1602.c angelegt
 * NFA-003: eUSCI_BO wird verwendet, um den I�C Bus anzusteuern
 *
 * Verdrahtung:
 *                MSP430FR2355     4k7  4k7     PCF8574
 *                    master        |    |
 *              -----------------   |    |
 *            -|XIN  P1.2/UCB0SDA|<-|----+->| SDA
 *             |                 |  |       |
 *            -|XOUT             |  |       |
 *             |     P1.3/UCB0SCL|<-+------>|SCL
 *    LEDred<--|P1.0             |          |
 *                                   3,3V-->| Vcc
 *
 *    Hinweis zu den pull-up Widerst�nden:
 *    Pr�fen, ob die Widerst�nde bereits auf dem PCF8574 board installiert sind (Standard).
 *    Ggf. m�ssen diese entfernt werden, wenn das Display nicht mit 3,3V, sondern mit 5V
 *    betrieben werden soll. Anstelle der entfernten SMD-Widerst�nde m�ssen dann externe
 *    pull-up Widerst�nde installiert werden, die nicht mit Vcc des Displays (5V), sondern
 *    mit Vcc des Launchpads (3,3V) verbunden sind. F�r die Steuerung des I2C Buses sollten
 *    in diesem Fall 3,3 V knapp ausreichend sein (> 0,7*Vcc), jedoch wird das Display hierbei
 *    knapp au�erhalb der Spezifikation betrieben.
 *
 */




/**
 * @mainpage Hauptseite des MSP430 Mikrocontroller-Projekts
 *
 *@author Mohamed Chahin Benyahya
 *
 *@date 03.07.2024
 *
 *Zweckbestimmung
 *---------------
 *
 * Dieses Projekt implementiert ein Steuerungssystem fuer ein 16x2 LCD-Display, LED und einen Servo-Motor
 * auf einem MSP430 Mikrocontroller. Die Anwendung reagiert auf Benutzereingaben ueber die beiden Tasten (S1/S2), um
 * die Servo-Position zu aendern und entsprechende Nachrichten auf dem LCD-Display anzuzeigen.
 *
 *Funktionsbeschreibung/bestimmungsgemae�er Gebrauch:
 *--------------------------------------------------
 *
 *Das System wird so konfiguriert, dass bei Betaetigung der Taste S1 sich der Servomotor 90 Grad (Gegenuhrzeigersinn) dreht und somit den
 *Schlossriegel nach hinten zieht. Gleichzeitig erfolgt ueber das LCD-Display eine visuelle Ausgabe: "Tuer offen" und die gruene LED
 *wird eingeschalten. Bei Betaetigung der Taste S2 dreht sich der Servomotor um 90 Grad (im Uhrzeigersinn), dadurch wird der Schlossriegel wieder
 *vorgeschoben. Gleichzeitig erfolgt ueber das LCD-Display eine visuelle Ausgabe: "Tuer geschlossen". Die gruene LED wird ausgeschalten
 *und die rote eingeschalten.
 *
 *
 *
 *
 *
 * @section features_sec Features
 *
 * - Initialisierung und Konfiguration von GPIO-Pins
 * - PWM-Steuerung fuer Servo-Motoren
 * - I2C-Kommunikation zur Ansteuerung eines LCD-Displays
 * - Timer-basierte Sleep-Funktion
 * - Benutzerinteraktion ueber Tasten
 * - LED-Anzeige basierend auf Tastenstatus
 *
 * @section structure_sec Projektstruktur
 *
 * Die Hauptfunktionen des Projekts sind:
 * - @ref init_gpio() - Initialisiert die GPIO-Pins
 * - @ref init_timer() - Initialisiert den Timer
 * - @ref init_cs() - Initialisiert das Clock-System
 * - @ref init_i2c() - Initialisiert die I2C-Schnittstelle
 * - @ref sleep() - Implementiert eine Sleep-Funktion
 * - @ref setServoPosition() - Setzt die Position des Servos
 *
 *
 * Beschreibung der Hardware:
 *  --------------------------
 * - MSP430fr2355
 * - HD44780 16x2 Display mit I2C Interface
 * - MG90S Micro Servomotor
 * - Logic Level Converter 8 Kanal
 * @code
 *                MSP430FR2355                Level                LCD16x2
 *                    master                  Converter
 *              -----------------            -----------         ----------
 *            -|XIN  P1.2/UCB0SDA|<-------->|A2       B2|<----->| SDA
 *             |                 |          |           |       |
 *            -|XOUT             |          |           |       |
 *             |     P1.3/UCB0SCL|<-------->|A1       B1|<----->|SCL
 *    LEDred<--|P1.0             |                              |
 *             |                 |                         5V-->| Vcc
 *  LEDgreen<--|P6.6             |
 *             |                 |
 *             |                 |
 *       SW1-->|P4.1             |          MG90S Servo
 *             |                 |           --------
 *       SW2-->|P2.3             |          |
 *             |            P1.6 |<-------->|PWM
 *             |                 |          |
 *             |                 |     5V-->|VCC
 * @endcode
 *
 *  Bilder des fertigen Systems:
 *  ----------------------------
 *
 * @image html final_system_1.jpg "Finales System 1"
 * @image rtf  final_system_1.jpg width=8cm
 *
 * @image html final_system_2.jpg "Finales System 2"
 * @image rtf  final_system_2.jpg width=8cm
 *
 *Beschreibung der Software
 *-------------------------
 *_Werkzeuge_
 *- Die Einstellung der Hardware Register des MSP430 erfolgt ueber TI-Grace.
 *- Die Entwicklung erfolgte unter TI CCS Ver. 12.7.1
 *- UML Diagramme wurden mit UMLet Version 15.0 erstellt
 *- Die Dokumentation wurde mit doxygen Version 1.11.0 erstellt
 *- Das Konfigurationsmanagement erfolgt ueber Git
 *
 *_Aufbau_
 *
 *
 * @image html haupt.jpg "Programmaufbau"
 * @image rtf  haupt.jpg width=8cm
 *
 *
 *@warning
 *- Beim Debugging vor dem Start der Debug Session unbedingt die Versorgungsspannung
 *  zum I2C-Modul unterbrechen, damit der Sensor neu gestartet wird. Andernfalls kann
 *  es passieren, dass sich der Sensor nicht ansprechen l�sst und das Programm somit
 *  keine Reaktion zeigt.
 *- Verbrauch des Servomotors kontrollieren. Bei Benutzung des Servomotors kann bei hohem
 *   Wiederstand die genutzte Spannung die Richtlinien f�r das MSP430 ueberschreiten
 *
 *
 *@todo
 *- main umschreiben von polling zu Interrupts mit sleepmode
 *- Implementierung des RFID-Sensors, ueber welchen das System fortan gesteuert werden soll--> nicht mehr ueber Tasten
 * @file main.c
 * @brief Hauptprogramm fuer das MSP430 Mikrocontroller-System zur Steuerung eines 16x2 LCD-Displays und eines Servos.
 */




/**
 * @file main.c
 *
 * @author Mohamed Chahin Benyahya
 *
 * @date 03.07.24
 *
 * Hauptprogramm fuer das MSP430 Mikrocontroller-System zur Steuerung eines 16x2 LCD-Displays, LED und eines Servos durch die zwei Tasten (S1/S2).
 *
 * Dieses Programm initialisiert die notwendigen Peripheriegeraete (GPIO, Timer, Clock System, I2C),
 * steuert ein LCD-Display, LED und ein Servo, und reagiert auf Benutzereingaben ueber Tasten.
 *
 * @details
 * - GPIO: Konfiguration der Ein- und Ausgaenge, LED-Steuerung und Tastenabfrage.
 * - Timer: Konfiguration und Nutzung zur PWM-Steuerung des Servos und zum Sleep-Management.
 * - Clock System: Initialisierung des Taktgebersystems, um die gewuenschte Frequenz bereitzustellen.
 * - I2C: Einrichtung der I2C-Schnittstelle zur Kommunikation mit dem LCD-Display.
 *
 *
 */

#include "driverlib.h"  // Treiberbibliothek f�r die Hardwarezugriffe
#include "Board.h"      // Board-spezifische Konfigurationen
#include "lcd1602.h"    // LCD-Bibliothek f�r ein 16x2 LCD-Display
#include <msp430.h>     // MSP430-spezifische Definitionsdatei
/**
 * @brief Definiert die gruene LED an Port6, Pin6
 */
#define GREEN_LED BIT6   // Gr�ne LED an P6.6
/**
 * @brief Definiert die rote LED an Port1, Pin0
 */
#define RED_LED BIT0     // Rote LED an P1.0
/**
 * @brief Definiert den Schalter S1 an Port4, Pin1
 */
#define BUTTON_S1 BIT1   // Schalter S1 an P4.1
/**
 * @brief Definiert den Schalter S2 an Port2, Pin3
 */
#define BUTTON_S2 BIT3   // Schalter S2 an P2.3

// Ziel-Frequenz f�r MCLK in kHz
/**
 * @brief Die Taktfrequenz des MSP wird auf den angegebenen Wert angepasst
 * @param CS_MCLK_DESIRED_FREQUENCY_IN_KHZ Die gewuenschte Taktfrequenz fuer MCLK in kHz.
 */
#define CS_MCLK_DESIRED_FREQUENCY_IN_KHZ 1000
// MCLK/FLLRef Verh�ltnis
/**
 * @brief Die Frequenz des MCLK wird um den Faktor des angebenen Wertes erhoeht
 * @param value CS_MCLK_DESIRED_FREQUENCY_IN_KHZ wird um den Faktor 30 erhoeht
 * @return Der um den Faktor 30 erhoehte Wert.
 */
#define CS_MCLK_FLLREF_RATIO 30

// Die 7-Bit Adresse des PCF8574-Bausteins: 0x3F (dezimal 63) : 0011 1111
// F�r Schreibzugriffe ergibt sich folgender Adresswert: 0111 1110 = 0x7E
// Die Driverlib erwartet jedoch die Angabe der tats�chlichen 7-Bit Adresse, also 0x3F
/**
 * @brief Definiert die 7-Bit Adresse des PCF8574-Baustein
 */
#define SLAVE_ADDRESS 0x3F
/**
 * @brief Die minimale Pulsbreite ist auf den angegebenen Wert geregelt
 * @param SERVO_MIN_PULSE_WIDTH Die gewuenschte Anzahl in Zyklen.
 */
#define SERVO_MIN_PULSE_WIDTH 1000   // Minimale Pulsbreite f�r den Servo (in Zyklen)
/**
 * @brief Die maximale Pulsbreite ist auf den angegebenen Wert geregelt
 * @param SERVO_MAX_PULSE_WIDTH Die gewuenschte Anzahl in Zyklen.
 */
#define SERVO_MAX_PULSE_WIDTH 2000   // Maximale Pulsbreite f�r den Servo (in Zyklen)
/**
 * @brief Initialisiert die GPIO-Pins.
 * @author Mohamed Chahin Benyahya
 *
 *@version        2
 *
 *  @date           _Version 2 (ab 03.07.2024)_
 *                  - geaendert: 2024-07-03
 *                    - Kommentierung der Quelldatei
 *                  - Test und Freigabe: 2024-07-03
 *                    - Doxygen-generierte Dateien vollstaendig
 *
 *  @date           _Version 1 (ab 2024-06-27)_
 *                  -erstellt: 2024-06-27
 *                    - Erstausgabe der Funktion
 *                  - Test und Freigabe: 2024-06-27
 *                    - Alle states werden fehlerfrei durchlaufen
 *
 *  @param [in,out] void
 *  @return         void
 *
 *  Zweck:
 *  ======
 *  -# Initalisierung der einzelnen GPIO komponenten
 *  -# LEDs, Buttons und Pin fuer die Kommunikation mit Servo
 *
 *  Umsetzung:
 *  ==========
 *  die beiden LEDs werden als Ausgang gesetzt und ausgeschalten
 *  die beiden Buttons werden als Eingang gesetzt und der Pull-Up Resistor f�r beide Tasten wird aktiviert und gesetzt
 *  die Pins 1.6 und 1.7 werden als Ausgang fuer die Kommunikation mit dem Servo gesetzt
 *
 *
 *  Initialisierung:
 *  ----------------
 *  nicht erforderlich
 *
 */
void init_gpio(void);

 /**
   * @brief Initialisiet den Timer
   * @author Mohamed Chahin Benyahya
   *
   *@version        2
   *
   *  @date           _Version 2 (ab 03.07.2024)_
   *                  - geaendert: 2024-07-03
   *                    - Kommentierung der Quelldatei
   *                  - Test und Freigabe: 2024-07-03
   *                    - Doxygen-generierte Dateien vollstaendig
   *
   *  @date           _Version 1 (ab 2024-06-27)_
   *                  -erstellt: 2024-06-27
   *                    - Erstausgabe der Funktion
   *                  - Test und Freigabe: 2024-06-27
   *                    - Alle states werden fehlerfrei durchlaufen
   *
   *  @param [in,out] void
   *  @return         void
   *
   *  Zweck:
   *  ======
   *  -# Konfiguration der Timer-Parameter
   *
   *
   *
   *  Umsetzung:
   *  ==========
   *  Es wird eine Auswahl der Taktquelle getroffen und -teiler.
   *  Zudem werden die Parameter f�r Taktperiode festgelegt.
   *  Au�erdem werden Spezifikation bez�glich der Interrups getroffen.
   *  Der Timer wird gestartet.
   *
   *
   *  Initialisierung:
   *  ----------------
   *  nicht erforderlich
   *
   */
void init_timer(void);

/**
  * @brief Initialisiet des Clock-Systems
  * @author Mohamed Chahin Benyahya
  *
  *@version        2
  *
  *  @date           _Version 2 (ab 03.07.2024)_
  *                  - geaendert: 2024-07-03
  *                    - Kommentierung der Quelldatei
  *                  - Test und Freigabe: 2024-07-03
  *                    - Doxygen-generierte Dateien vollstaendig
  *
  *  @date           _Version 1 (ab 2024-06-27)_
  *                  -erstellt: 2024-06-27
  *                    - Erstausgabe der Funktion
  *                  - Test und Freigabe: 2024-06-27
  *                    - Alle states werden fehlerfrei durchlaufen
  *
  *  @param [in,out] void
  *  @return         void
  *
  *  Zweck:
  *  ======
  *  -# Clock-System korrekt initialisieren durch ausgewaehlte Taktquellen
  *  -# Sicherstellung das Clock-System richtig funktioniert bei Oszillatorfehlern
  *
  *
  *
  *  Umsetzung:
  *  ==========
  *  Clock-Systeme werden auf den REFOCLK gelegt fuer stabilitaet und konfiguriert sind.
  *  Speicherung der von Trim-Werte und Parameter zu speichern.
  *  Aktivierung der Interrupts fuer Oszillatoren bei fehlern.
  *
  *  Initialisierung:
  *  ----------------
  *  nicht erforderlich
  *
  */
void init_cs(void);

void init_i2c(void);

/**
 * @brief Wartet fuer die angegebene Anzahl von Millisekunden.
 * @param ms Anzahl der Millisekunden.
 */
void sleep(uint16_t ms);

/**
 * @brief Setzt die Position des Servos.
 * @param position Pulsbreite fuer den Servo (in Zyklen).
 */
void setServoPosition(unsigned int position);

volatile uint16_t sleep_count = 0;

/**
  * @brief Hauptfunktion des Systems
  * @author Mohamed Chahin Benyahya
  *
  *@version        2
  *
  *  @date           _Version 2 (ab 03.07.2024)_
  *                  - geaendert: 2024-07-03
  *                    - Kommentierung der Quelldatei
  *                  - Test und Freigabe: 2024-07-03
  *                    - Doxygen-generierte Dateien vollstaendig
  *
  *  @date           _Version 1 (ab 2024-06-27)_
  *                  -erstellt: 2024-06-27
  *                    - Erstausgabe der Funktion
  *                  - Test und Freigabe: 2024-06-27
  *                    - Alle states werden fehlerfrei durchlaufen
  *
  *  @param [in,out] void
  *  @return         void
  *
  *  Zweck:
  *  ======
  *  -#ist fuer die Ausf�hrung des Systems und deren Reihenfolge verantwortlich
  *
  *  Umsetzung:
  *  ==========
  *  Initialisierung der verschiedenen Komponeten wird ausgef�hrt und die GPIO Mode wird angepasst
  *  Beinhaltet die die Hauptschleife: Bei Tastendruck wird ein Ablauf ausgefuehrt, welcher je nach Taste die
  *  Position des Servomotors anpasst, die Ausgabe des LCD-Display aendert und die aktive LED tauscht
  *  Taste S1 als Oeffnungsmechanismus und Taste S2 als Schlie�mechanismus
  *
  *
  *  Initialisierung:
  *  ----------------
  *  nicht erforderlich
  *
  */
void main(void)
{
    WDT_A_hold(WDT_A_BASE); // Watchdog-Timer stoppen
    init_gpio();            // GPIO initialisieren
    init_timer();           // Timer initialisieren
    init_cs();              // Clock-System initialisieren
    init_i2c();             // I2C-Schnittstelle initialisieren
    __bis_SR_register(GIE); // Globale Interrupts aktivieren

    lcd1602_init();         // LCD initialisieren
    lcd1602_backlight(true); // Hintergrundbeleuchtung des LCDs einschalten

    // GPIO f�r den Servo konfigurieren
    P1DIR |= BIT6 | BIT7;   // P1.6 und P1.7 als Ausgang setzen
    P1SEL1 |= BIT6 | BIT7;  // Optionen f�r P1.6 und P1.7 w�hlen

    // GPIO Power-on Default High-Impedance Mode deaktivieren, um vorher konfigurierte Port-Einstellungen zu aktivieren
    PM5CTL0 &= ~LOCKLPM5;

    TB0CCR0 = 20000 - 1;     // PWM-Periode setzen
    TB0CCTL1 = OUTMOD_7;     // CCR1 zur�cksetzen/setzen
    TB0CCR1 = 1500;          // Anfangsposition (1,5 ms Pulsbreite)
    TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR; // SMCLK w�hlen, Up-Modus, TBR l�schen

    while (1)
    {
        if (GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN1) == GPIO_INPUT_PIN_LOW)
        {
            lcd1602_clear();

            setServoPosition(SERVO_MAX_PULSE_WIDTH); // Servo auf 90 Grad Position setzen
            P6OUT |= GREEN_LED;  // Gr�ne LED einschalten
            P1OUT &= ~RED_LED;   // Rote LED ausschalten
            lcd1602_write(1, "HuHu!!");
            lcd1602_write(2, "Tuer offen");
            sleep(25);           // 25 Millisekunden warten
        }
        else if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN3) == GPIO_INPUT_PIN_LOW)
        {
            lcd1602_clear();

            setServoPosition(SERVO_MIN_PULSE_WIDTH); // Servo auf Ausgangsposition setzen
            P6OUT &= ~GREEN_LED; // Gr�ne LED ausschalten
            P1OUT |= RED_LED;    // Rote LED einschalten
            lcd1602_write(1, "hasta la vista!!");
            lcd1602_write(2, "Tuer geschlossen");
            sleep(25);           // 25 Millisekunden warten
        }
        sleep(12);  // 12 Millisekunden warten
    }
}

/**
 * @brief Initialisiert die GPIO-Pins.
 */
void init_gpio(void)
{
    P6DIR |= GREEN_LED; // Gr�ne LED als Ausgang setzen
    P1DIR |= RED_LED;   // Rote LED als Ausgang setzen

    P4DIR &= ~BUTTON_S1; // Schalter S1 (P4.1) als Eingang setzen
    P4REN |= BUTTON_S1;  // Pull-Up Widerstand f�r Schalter S1 aktivieren
    P4OUT |= BUTTON_S1;  // Pull-Up Widerstand f�r Schalter S1 setzen

    P2DIR &= ~BUTTON_S2; // Schalter S2 (P2.3) als Eingang setzen
    P2REN |= BUTTON_S2;  // Pull-Up Widerstand f�r Schalter S2 aktivieren
    P2OUT |= BUTTON_S2;  // Pull-Up Widerstand f�r Schalter S2 setzen

    // GPIO f�r den Servo konfigurieren
    P1DIR |= BIT6 | BIT7;   // P1.6 und P1.7 als Ausgang setzen
    P1SEL1 |= BIT6 | BIT7;  // Optionen f�r P1.6 und P1.7 w�hlen

    // Anfangszustand f�r LEDs setzen
    P6OUT &= ~GREEN_LED; // Gr�ne LED ausschalten
    P1OUT &= ~RED_LED;   // Rote LED ausschalten

    // GPIO Power-on Default High-Impedance Mode deaktivieren, um vorher konfigurierte Port-Einstellungen zu aktivieren
    PM5CTL0 &= ~LOCKLPM5;

    // LED1 als Ausgang setzen
    GPIO_setAsOutputPin(GPIO_PORT_LED1, GPIO_PIN_LED1);

    // P4.1 (S1) und P2.3 (S2) als Eing�nge konfigurieren
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN1);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN3);

    PMM_unlockLPM5(); // LPM5 freigeben
}

/**
 * @brief Initialisiert den Timer.
 */
void init_timer(void)
{
    static Timer_B_initUpModeParam param = {0};

    param.clockSource = TIMER_B_CLOCKSOURCE_SMCLK; // Taktquelle: SMCLK
    param.clockSourceDivider = TIMER_B_CLOCKSOURCE_DIVIDER_1; // Taktteiler: 1
    param.timerPeriod = 999; // Timer-Periode: 999 -> Interrupt alle 1000 Taktimpulse
    param.timerInterruptEnable_TBIE = TIMER_B_TBIE_INTERRUPT_DISABLE; // Kein Interrupt auf 0x0000
    param.captureCompareInterruptEnable_CCR0_CCIE = TIMER_B_CAPTURECOMPARE_INTERRUPT_ENABLE; // Interrupt auf TRmax
    param.timerClear = TIMER_B_DO_CLEAR;
    param.startTimer = true;

    // Timer starten
    Timer_B_initUpMode(TB0_BASE, &param);
}

/**
 * @brief Initialisiert das Clock-System.
 */
void init_cs(void)
{
    // DCO FLL-Referenz auf REFO setzen
    CS_initClockSignal(CS_FLLREF, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    // ACLK auf REFO setzen
    CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    // Struct-Variable zum Speichern der Software-Trim-Werte erstellen
    CS_initFLLParam param = {0};

    // Verh�ltnis/Erw�nschte MCLK-Frequenz setzen, DCO initialisieren, Trim-Werte speichern
    CS_initFLLCalculateTrim(CS_MCLK_DESIRED_FREQUENCY_IN_KHZ, CS_MCLK_FLLREF_RATIO, &param);

    // Alle OSC-Fehlerflaggen l�schen
    CS_clearAllOscFlagsWithTimeout(1000);

    // Oszillator-Fehler-Interrupt aktivieren
    SFR_enableInterrupt(SFR_OSCILLATOR_FAULT_INTERRUPT);
}

/**
 * @brief Initialisiet die I2C-Schnittstelle.
 * @author Mohamed Chahin Benyahya
 *
 *@version        2
 *
 *  @date           _Version 2 (ab 03.07.2024)_
 *                  - geaendert: 2024-07-03
 *                    - Kommentierung der Quelldatei
 *                  - Test und Freigabe: 2024-07-03
 *                    - Doxygen-generierte Dateien vollstaendig
 *
 *  @date           _Version 1 (ab 2024-06-27)_
 *                  -erstellt: 2024-06-27
 *                    - Erstausgabe der Funktion
 *                  - Test und Freigabe: 2024-06-27
 *                    - Alle states werden fehlerfrei durchlaufen
 *
 *  @param [in,out] void
 *  @return         void
 *
 *  Zweck:
 *  ======
 *  -# Initalisierung die I2C Schnittstellen
 *  -# Konfiguration der I2C-Pins(SCL und SDA) und die I2C-Parameter setzen
 *
 *
 *
 *  Umsetzung:
 *  ==========
 *  Pin 1.2 und 1.3 werden f�r die Kommunikation mit dem LCD-Display konfiguriert.
 *  Clock-Source, Taktfrequenz und Datenrate wird gesetzt.
 *  Slave-Adresse wird gesetzt und der Master initalisiert und der Modus der Verbindung wird auf Sendemodus vom MSP gesetzt.
 *
 *  Initialisierung:
 *  ----------------
 *  nicht erforderlich
 *
 */
void init_i2c(void)
{
    EUSCI_B_I2C_initMasterParam param = {0};

    // Pins f�r I2C konfigurieren
    /*
    * Port 1 ausw�hlen
    * Pin 2, 3 als Eingang mit Funktion waehlen (UCB0SIMO/UCB0SDA, UCB0SOMI/UCB0SCL).
    */

    // 1 ausw�hlen fuer UCB0SCL und UCB0SDA (SCL und SDA)
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_UCB0SCL, GPIO_PIN_UCB0SCL, GPIO_FUNCTION_UCB0SCL);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_UCB0SDA, GPIO_PIN_UCB0SDA, GPIO_FUNCTION_UCB0SDA);

    param.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK; // Clock-Source w�hlen: SMCLK
    param.i2cClk = CS_getSMCLK(); // I2C-Taktfrequenz setzen
    param.dataRate = EUSCI_B_I2C_SET_DATA_RATE_100KBPS; // Datenrate: 100 kbps
    param.byteCounterThreshold = 1; // Byte-Counter-Schwelle: 1
    param.autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP; // Kein automatisches STOP erzeugen
    EUSCI_B_I2C_initMaster(EUSCI_B0_BASE, &param); // I2C-Master initialisieren

    // Slave-Adresse setzen
    EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, SLAVE_ADDRESS);
    // Im Sendemodus setzen
    EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
    // I2C-Modul aktivieren, um Operationen zu starten
    EUSCI_B_I2C_enable(EUSCI_B0_BASE);
}

/**
 * @brief Wartet fuer die angegebene Anzahl von Millisekunden.
 *
 * Die Funktion kehrt erst dann zurueck, wenn Timer0_B3 die angegebene Anzahl von
 * ms absolviert hat. Unbedingt beachten: Diese einfache Implementierung eines
 * sleep-Timers funktioniert nur, solange kein Interrupt nesting verwendet wird.
 * Waehrend der sleep-Perioden des Timer koennen andere Interrupt-Routinen ausgefuehrt
 * werden; diese duerfen aber nicht in die main loop zurueckkehren, sondern muessen
 * den aktuellen sleep mode beim Verlassen der ISR wieder herstellen.
 *
 * @param ms Anzahl der Millisekunden.
 */
void sleep(uint16_t ms)
{
    sleep_count = ms;
    while (sleep_count > 0) {
        __bis_SR_register(LPM0_bits + GIE);  // In LPM0 mit aktivierten Interrupts wechseln
        __no_operation();  // F�r Debugger
    }
}

/**
 * @brief Setzt die Postion des Servos
 * @author Mohamed Chahin Benyahya
 *
 *@version        2
 *
 *  @date           _Version 2 (ab 03.07.2024)_
 *                  - geaendert: 2024-07-03
 *                    - Kommentierung der Quelldatei
 *                  - Test und Freigabe: 2024-07-03
 *                    - Doxygen-generierte Dateien vollstaendig
 *
 *  @date           _Version 1 (ab 2024-06-27)_
 *                  -erstellt: 2024-06-27
 *                    - Erstausgabe der Funktion
 *                  - Test und Freigabe: 2024-06-27
 *                    - Alle states werden fehlerfrei durchlaufen
 *
 *  @param [in]     int f�r die Pulsbreite
 *  @return         void
 *
 *  Zweck:
 *  ======
 *  -# Bewegung des Servomotors in einen gewissen Zustand
 *
 *
 *
 *  Umsetzung:
 *  ==========
 *  Der Funktion wird ein int-Wert uebergeben, dieser Wert dient zur Bestimmung der neuen Postion. Der Motor wird bei niedrigem Wert auf die Position mit minimaler Pulsbreite gedreht.
 *  Bei einem hohen Wert wird der Motor auf die Position mit maximaler Pulsbreite gedreht.
 *
 *  Initialisierung:
 *  ----------------
 *  SERVO_MIN_PULSE_WIDTH
 *  SERVO_MAX_PULSE_WIDTH
 */
void setServoPosition(unsigned int position)
{
    if (position < SERVO_MIN_PULSE_WIDTH)
        position = SERVO_MIN_PULSE_WIDTH;
    else if (position > SERVO_MAX_PULSE_WIDTH)
        position = SERVO_MAX_PULSE_WIDTH;

    TB0CCR1 = position; // Timer B0 CCR1 auf die angegebene Position setzen
}

/**
 * @brief Nicht maskierbarer Interrupt (NMI) ISR.
 */
#pragma vector = UNMI_VECTOR
__interrupt void NMI_ISR(void)
{
    uint16_t status;
    do
    {
        // Falls die Oszillator-Fehlerflaggen nach dem Timeout immer noch nicht gel�scht werden k�nnen,
        // hier verweilen und warten.
        status = CS_clearAllOscFlagsWithTimeout(1000);
    } while (status != 0);
}

/**
 * @brief TimerB0 Interrupt-Vektor (TBxIV) Handler.
 */
#pragma vector = TIMER0_B0_VECTOR
__interrupt void TIMER0_B0_ISR(void)
{
    if (sleep_count > 0) {
        sleep_count--;
        if (sleep_count == 0) {
            __bic_SR_register_on_exit(LPM0_bits);  // Aus LPM0 herausgehen
        }
    }
}
