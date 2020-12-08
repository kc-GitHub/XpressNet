/*
  XpressNet.h - library for XpressNet protocoll
  Copyright (c) 2013 Philipp Gahtow  All right reserved.

  Version 1.5

  Notice:
  Works until now, only with XPressNet Version 3.0 or higher!
*/

// ensure this library description is only included once
#ifndef XpressNet_h
#define XpressNet_h

// include types & constants of Wiring core API
#if defined(WIRING)
#include <Wiring.h>
#elif ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include "app_cfg.h"

/* From the ATMega datasheet: */
//--------------------------------------------------------------------------------------------
// Which serial port is used, if we have more than one on the chip?
// note that the 328s (the currently produced "smaller" chips) only
// have one serial port, so we force this.
#if APP_CFG_UC == APP_CFG_UC_ATMEL
#ifdef __AVR_ATmega328P__
#define SERIAL_PORT_0
#undef SERIAL_PORT_1
#else
// Maybe we are running on a MEGA chip with more than 1 port? If so, you
// can put the serial port to port 1, and use the port 0 for status messages
// to your PC.
#define SERIAL_PORT_1
#undef SERIAL_PORT_0
#endif
#else
#endif

// when sending data, do NOT continue until the hardware has sent the data out
#if APP_CFG_UC == APP_CFG_UC_ATMEL
#ifdef __AVR_ATmega8__
#define WAIT_FOR_XMIT_COMPLETE                                                                                         \
    {                                                                                                                  \
        while (!(UCSRA & (1 << TXC)))                                                                                  \
            ;                                                                                                          \
        UCSRA = (1 << TXC);                                                                                            \
        UCSRA = 0;                                                                                                     \
    }
#else
#ifdef SERIAL_PORT_0
#define WAIT_FOR_XMIT_COMPLETE                                                                                         \
    {                                                                                                                  \
        while (!(UCSR0A & (1 << TXC0)))                                                                                \
            ;                                                                                                          \
        UCSR0A = (1 << TXC0);                                                                                          \
        UCSR0A = 0;                                                                                                    \
    }
#else
#define WAIT_FOR_XMIT_COMPLETE                                                                                         \
    {                                                                                                                  \
        while (!(UCSR1A & (1 << TXC1)))                                                                                \
            ;                                                                                                          \
        UCSR1A = (1 << TXC1);                                                                                          \
        UCSR1A = 0;                                                                                                    \
    }
#endif
#endif
#else
#define WAIT_FOR_XMIT_COMPLETE
#endif

//--------------------------------------------------------------------------------------------

// XPressnet Call Bytes.
// broadcast to everyone, we save the incoming data and process it later.
#define GENERAL_BROADCAST 0x160

// certain global XPressnet status indicators
#define csNormal 0x00          // Normal Operation Resumed ist eingeschaltet
#define csEmergencyStop 0x01   // Der Nothalt ist eingeschaltet
#define csTrackVoltageOff 0x02 // Die Gleisspannung ist abgeschaltet
#define csShortCircuit 0x04    // Kurzschluss
#define csServiceMode 0x20     // Der Programmiermodus ist aktiv - Service Mode

#define csLocDataBaseSend 0x40

// XpressNet Befehl, jedes gesendete Byte
#define XNetlength 0 // Länge
#define XNetmsg 1    // Message
#define XNetcom 2    // Kennung/Befehl
#define XNetdata1 3  // Databyte1
#define XNetdata2 4  // Databyte2
#define XNetdata3 5  // Databyte3
#define XNetdata4 6  // Databyte4
#define XNetdata5 7  // Databyte5

typedef struct // Lokdaten	(Lok Events)
{
    uint8_t low;   // A7, A6, A5, A4, A3, A2, A1, A0
    uint8_t high;  // 0, 0, A13, A12, A11, A10, A9, A8 -> DFAA AAAA
    uint8_t mode;  // Kennung 0000 B0FF -> B=Busy(1), F=Fahrstufen (0=14, 1=27, 2=28, 3=128)
    uint8_t speed; // 0, Speed 0..127 (0x00 - 0x7F) -> 0SSS SSSS
    uint8_t f0;    // X X Dir F0 F4 F3 F2 F1
    uint8_t f1;    // F12 F11 F10 F9 F8 F7 F6 F5
    uint8_t f2;    // F20 F19 F18 F17 F16 F15 F14 F13
    uint8_t f3;    // F28 F27 F26 F25 F24 F23 F22 F21
    uint8_t state; // Zahl der Zugriffe
} XNetLok;

typedef enum
{
    recieveStatusNone = 0,
    recieveStatusPower
} recieveStatus;

#define XSendMaxData 8 // Anzahl Elm im Data Array XSend
typedef struct         // Antwort/Abfragespeicher
{
    uint8_t length;          // Speicher für Datenlänge
    byte data[XSendMaxData]; // zu sendende Daten
} XSend;

/* Slotliste Loks */
#define XSendMax 16      // Maximalanzahl Daten im Sendepuffer
#define SlotMax 15       // Slots für Lokdaten
#define SlotInterval 100 // Zeitintervall zur Aktualisierung der Slots (ms)

// library interface description
class XpressNetClass
{
    // user-accessible "public" interface
public:
    XpressNetClass(void);                // Constuctor
    void start(byte XAdr, int XControl); // Initialisierung Serial
    recieveStatus receive(void);         // Prüfe ob XNet Packet vorhanden und werte es aus.

    bool setPower(byte Power); // Zustand Gleisspannung Melden
    byte getPower();           // Zusand Gleisspannung geben
    void setHalt();            // Zustand Halt Melden
    bool commandStationStatusRequest();
    bool getLocoInfo(byte Adr_High, byte Adr_Low); // Abfragen der Lokdaten (mit F0 bis F12)
    bool getLocoFunc(byte Adr_High, byte Adr_Low); // Abfragen der Lok Funktionszustände F13 bis F28
    bool setLocoHalt(byte Adr_High, byte Adr_Low); // Lok anhalten
    bool setLocoDrive(byte Adr_High, byte Adr_Low, uint8_t Steps, uint8_t SpeedDir); // Lokdaten setzten
    bool setLocoFunc(byte Adr_High, byte Adr_Low, uint8_t type, uint8_t fkt);        // Lokfunktion setzten
    void getLocoStateFull(byte Adr_High, byte Adr_Low, bool Anfrage);                // Gibt Zustand der Lok zurück.
    bool getTrntInfo(byte FAdr_High, byte FAdr_Low);          // Ermitteln der Schaltstellung einer Weiche
    bool setTrntPos(byte FAdr_High, byte FAdr_Low, byte Pos); // Schalten einer Weiche
    // Programming:
    void readCVMode(byte CV);                                         // Lesen der CV im CV-Mode
    void writeCVMode(byte CV, byte Data);                             // Schreiben einer CV im CV-Mode
    void getresultCV();                                               // Programmierergebnis anfordern
    void writeCvPom(byte Adr_High, byte Adr_Low, word CV, byte Data); // Schreiben einer CV om POM Mode
    void TransmitLocDatabaseEnable();                                 // Start senden loc data base Daten
    void TransmitLocDatabaseDisable();                                // Stop senden loc data base Daten
    void TransmitLocData(byte Adr_High, byte Adr_Low, byte locCount, byte numOfLocs);
    // Slot:
    void setFree(byte Adr_High, byte Adr_Low); // Lok aus Slot nehmen

    // public only for easy access by interrupt handlers
    static inline void handle_interrupt(uint16_t DataRx); // Serial Interrupt bearbeiten
    static inline void XpNetMsgEnd(void);

    // library-accessible "private" interface
private:
    // Variables:
    boolean XNetRun;                      // XpressNet ist aktiv
    byte MY_ADDRESS;                      // XpressNet address: must be in range of 1-31; must be unique.
    byte MAX485_CONTROL;                  // Port for send or receive control
    unsigned int myDirectedOps;           // the address we look for when we are listening for ops
    unsigned int myCallByteInquiry;       // the address we look for for our Call Byte Window
    unsigned int myRequestAck;            // the address for a request acknowlegement sent
    unsigned int XNetMsg[15];             // Serial receive (Length, Message, Command, Data1 to Data5)
    boolean XNetTrasnmitLocDbData;        // Transmit loc database
    boolean ReadData;                     // Empfangene Serial Daten: (Speichern = true/Komplett = false)
    static XpressNetClass* active_object; // aktuelle aktive Object
    void XNetget(uint16_t DataRx);        // Empfangene Daten eintragen
    XSend XNetSend[XSendMax];             // Sendbuffer
    XNetLok xLokSts[SlotMax];             // Speicher für aktive Lokzustände

    // Functions:
    void getXOR(unsigned char* data, byte length); // calculate the XOR
    unsigned int callByteParity(unsigned int me);  // calculate the parity bit
    int USART_Receive(void);                       // Serial Empfangen
    void USART_Transmit(unsigned char data8);      // Serial Senden
    void XNetclear(void);                          // Serial Nachricht zurücksetzten

    void XNetclearSendBuf();                                       // Sendbuffer leeren
    boolean XNetSendadd(unsigned char* dataString, int byteCount); // Zum Sendebuffer Hinzufügen
    void XNetsend(void);                                           // Send Saved Data aus Sendebuffer
    void XNetsend(unsigned char* dataString, int byteCount);       // Sende Daten aus Array

    // Adressrequest:
    int ReqLocoAdr; // Adresse für die Lok Daten angefragt wurden
    int ReqLocoAgain;
    int ReqFktAdr; // Adresse für die F2 und F3 angefragt wurde

    // SlotServer:
    long SlotTime;                                                           // store last time the Slot ask
    int SlotLast;                                                            // letzter bearbeiteter Slot
    void UpdateBusySlot(void);                                               // Fragt Zentrale nach aktuellen Zuständen
    void xLokStsclear(void);                                                 // löscht alle Slots
    bool xLokStsadd(byte MSB, byte LSB, byte Mode, byte Speed, byte FktSts); // Eintragen Änderung / neuer Slot XLok
    bool xLokStsFunc0(byte MSB, byte LSB, byte Func);                        // Eintragen Änderung / neuer Slot XFunc0
    bool xLokStsFunc1(byte MSB, byte LSB, byte Func1);                       // Eintragen Änderung / neuer Slot XFunc1
    bool xLokStsFunc23(byte MSB, byte LSB, byte Func2, byte Func3);          // Eintragen Änderung / neuer Slot XFunc23
    bool xLokStsBusy(byte Slot);                                             // Busy Bit Abfragen
    void XLokStsSetBusy(byte MSB, byte LSB);                                 // Lok Busy setzten
    byte xLokStsgetSlot(byte MSB, byte LSB); // gibt Slot für Adresse zurück / erzeugt neuen Slot (0..126)
    int xLokStsgetAdr(byte Slot);            // gibt Lokadresse des Slot zurück, wenn 0x0000 dann keine Lok vorhanden
    bool xLokStsIsEmpty(byte Slot);          // prüft ob Datenpacket/Slot leer ist?
    void xLokStsSetNew(byte Slot, byte MSB, byte LSB); // Neue Lok eintragen mit Adresse
    byte getNextSlot(byte Slot);                       // gibt nächsten genutzten Slot

    // Spannung und GO/STOP Events:
    byte Railpower; // Gleisspannung

    // Programming:

    // Lok Status:

    // Funktionen

    // Status LED:
    int ledState;        // ledState used to set the LED
    long previousMillis; // will store last time LED was updated
};

#if defined(__cplusplus)
extern "C"
{
#endif

    extern void notifyXNetStatus(uint8_t LedState) __attribute__((weak));
    extern void notifyXNetVer(uint8_t V, uint8_t ID) __attribute__((weak));
    extern void notifyXNetPower(uint8_t State) __attribute__((weak));
    extern void NotifyXNet(uint8_t Data) __attribute__((weak));
    extern void notifyLokFunc(uint8_t Adr_High, uint8_t Adr_Low, uint8_t F2, uint8_t F3) __attribute__((weak));
    extern void notifyLokAll(uint8_t Adr_High, uint8_t Adr_Low, boolean Busy, uint8_t Steps, uint8_t Speed,
        uint8_t Direction, uint8_t F0, uint8_t F1, uint8_t F2, uint8_t F3, boolean Req) __attribute__((weak));
    extern void notifyLokDataBaseDataReceive(
        uint8_t Adr_High, uint8_t Adr_Low, uint8_t LocCount, uint8_t NumberOfLocs, char* LocName) __attribute__((weak));
    extern void notifyCVInfo(uint8_t State) __attribute__((weak));
    extern void notifyCVResult(uint8_t cvAdr, uint8_t cvData) __attribute__((weak));
    extern void notifyTrnt(uint8_t Adr_High, uint8_t Adr_Low, uint8_t Pos) __attribute__((weak));
    extern void Stm32Uart2Int(uint16_t DataRx);
    extern void Stm32UartTxEnd(void);

    //	extern void notifyXNetData(unsigned int data, bool line) __attribute__((weak));

#if defined(__cplusplus)
}
#endif

#endif
