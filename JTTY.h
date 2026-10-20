// JTTY.h
//
// JTTY transmit support for TXlink4_Nano.
//
// JTTY frame = 13 sync symbols + 46 data symbols = 59 symbols, 4-GFSK,
// 31.25 Hz tone spacing, 32 ms per symbol (1.888 s per frame), 1..16 frames.
// No T/R periods: a transmission starts immediately and runs to its end.
//
// Serial protocol (PC -> Nano):
//   {M16}        select JTTY mode (once)
//   {J<hex>}     message data symbols ONLY (sync is inserted here), 23 hex
//                digits per frame. Each hex digit = 2 symbols: s0 | (s1 << 2).
//                Sent while idle = new message; sent while transmitting JTTY =
//                appended (gapless chaining).
//   {X1}         start (always send after {J..}; harmless if already on)
//   {X0}         abort immediately
//
// Storage: data symbols are packed 4 per byte into the RTTY queue buffer
// (message_buffer, 288 bytes), which is unused in JTTY mode. 16 frames need
// 16*46/4 = 184 bytes. No extra RAM is used.

#ifndef JTTY_H
#define JTTY_H

#include <Arduino.h>

// --- Mode and modulation constants ------------------------------------------
#define MODE_JTTY_VALUE                  16     // add MODE_JTTY = 16 to the mode enum
#define JTTY_TONE_SPACING                3125   // 31.25 Hz (Hz x 100)
#define JTTY_tone_spacing_FTWincr_x100   74565  // 31.25 / 0.0419095 Hz per bit, x100
#define JTTY_tone_duration               32     // ms per symbol (12000/384 = 31.25 baud)

// --- Frame layout --------------------------------------------------------------
#define JTTY_SYNC_LEN      13
#define JTTY_DATA_LEN      46
#define JTTY_FRAME_LEN     (JTTY_SYNC_LEN + JTTY_DATA_LEN)   // 59
#define JTTY_MAX_FRAMES    16
#define JTTY_MAX_DATA      (JTTY_MAX_FRAMES * JTTY_DATA_LEN)  // 736 symbols
#define JTTY_BUF_BYTES     (JTTY_MAX_DATA / 4)                // 184 bytes

// --- State (defined in JTTY.ino) -------------------------------------------------
extern uint16_t jttyDataCount;     // data symbols published (whole frames only)
extern bool     jttyRxActive;      // true while inside a {J...} command

// --- Serial receive (called from ProcessSerialIn) --------------------------------
void jttyRxBegin();                // 'J' seen as first char after '{'
void jttyRxChar(char c);           // each payload char (hex digit)
void jttyRxEnd();                  // closing '}' : publish complete frames

// --- Transmit ---------------------------------------------------------------------
void     setJttyDefaults();        // from setDigDefaults() case MODE_JTTY
void     transmitJtty();           // from DigitalTXmsg() when TxMode == MODE_JTTY
uint8_t  jttyTone(uint16_t i);     // tone 0..3 for channel symbol i (sync inserted)
uint16_t jttyTotalSymbols();       // channel symbols currently available
void     jttyReset();              // clear message (mode change, {X0}, end of TX)

#endif
