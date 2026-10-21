// JTTY.ino
//
// JTTY transmit for TXlink4_Nano. See JTTY.h for the serial protocol.
// Arduino concatenates all .ino files, so the globals and functions of
// TXlink4_Nano.ino (dds, CurFreq, FTxTxDF, DigiTxOn, tone_duration,
// tone_spacing_FTWincr_x100, gTran*, SWRmeasure(), ProcessSerialIn(),
// gaussianTransition()) are visible here.

#include "JTTY.h"
#include "rtty_queue.h"

// Packed data symbols live in the RTTY queue buffer (unused in JTTY mode)
#define JTTY_BUF ((uint8_t*)message_buffer)

static_assert(sizeof(message_buffer) >= JTTY_BUF_BYTES,
              "RTTY message_buffer too small for JTTY data");

// Sync vector is13 from WSJT-X lib/jtty/jtty_fec_mod.f90
const uint8_t jttySync[JTTY_SYNC_LEN] PROGMEM = {0,2,2,3,0,0,3,2,1,3,1,2,0};

uint16_t jttyDataCount = 0;        // published data symbols (multiple of 46)
bool     jttyRxActive  = false;
bool     jttyTune      = false;
static uint16_t jttyRxPos;         // next data symbol to write
static bool     jttyRxBad;         // bad char or overflow in this command


// ---------------------------------------------------------------------------
// Serial receive: hex digits are decoded straight into the packed buffer as
// they arrive, so no ASCII copy of the message is ever held.
// Symbol d is stored at byte d>>2, bits 2*(d&3).
// ---------------------------------------------------------------------------

static uint8_t jttyHexVal(char c)
{
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return 0xFF;
}

static void jttyPutSymbol(uint16_t d, uint8_t s)
{
  uint8_t *b  = &JTTY_BUF[d >> 2];
  uint8_t  sh = (d & 3) * 2;
  *b = (*b & ~(3 << sh)) | ((s & 3) << sh);
}

void jttyRxBegin()
{
  jttyRxActive = true;
  jttyRxBad    = false;
  if (DigiTxOn && TxMode == MODE_JTTY && jttyDataCount > 0) {
    jttyRxPos = jttyDataCount;                 // append behind current message
  } else {
    jttyDataCount = 0;                         // new message
    jttyRxPos     = 0;
    jttyTune      = false;
  }
}

void jttyRxChar(char c)
{
  if (jttyRxBad) return;
  if (c == 'T' && jttyRxPos == 0 && jttyDataCount == 0) {   // {JT} = tune
    jttyTune = true;
    return;
  }
  if (jttyTune) { jttyRxBad = true; return; }  // nothing may follow T
  uint8_t v = jttyHexVal(c);
  if (v == 0xFF || jttyRxPos + 2 > JTTY_MAX_DATA) { jttyRxBad = true; return; }
  jttyPutSymbol(jttyRxPos++, v & 3);
  jttyPutSymbol(jttyRxPos++, v >> 2);
}

void jttyRxEnd()
{
  jttyRxActive = false;
  // Publish only whole frames, so the TX loop never sees a half-written one
  if (!jttyRxBad && jttyRxPos > jttyDataCount && (jttyRxPos % JTTY_DATA_LEN) == 0) {
    jttyDataCount = jttyRxPos;
  }
  // else: rejected - leave jttyDataCount unchanged
  if (jttyRxBad) jttyTune = false;             // malformed {JT...} is not a tune request
  // Optional ack for the PC log, e.g. {jN} with N = frames now held:
  // Serial.print("{j"); Serial.print(jttyDataCount / JTTY_DATA_LEN); Serial.print('}');
}


// ---------------------------------------------------------------------------
// Symbol access
// ---------------------------------------------------------------------------

uint16_t jttyTotalSymbols()
{
  return (jttyDataCount / JTTY_DATA_LEN) * JTTY_FRAME_LEN;
}

uint8_t jttyTone(uint16_t i)
{
  uint8_t f = i % JTTY_FRAME_LEN;
  if (f < JTTY_SYNC_LEN) return pgm_read_byte(&jttySync[f]);
  uint16_t d = (i / JTTY_FRAME_LEN) * JTTY_DATA_LEN + (f - JTTY_SYNC_LEN);
  return (JTTY_BUF[d >> 2] >> ((d & 3) * 2)) & 3;
}

void jttyReset()
{
  jttyDataCount = 0;
  jttyRxActive  = false;
  jttyRxPos     = 0;
  jttyTune      = false;
}


// ---------------------------------------------------------------------------
// Mode defaults (called from setDigDefaults)
// ---------------------------------------------------------------------------

void setJttyDefaults()
{
  symbol_count              = 0;                          // not used by JTTY
  tone_spacingHz_x100       = JTTY_TONE_SPACING;
  tone_spacing_FTWincr_x100 = JTTY_tone_spacing_FTWincr_x100;
  tone_duration             = JTTY_tone_duration;         // 32 ms

  // Gaussian transitions: BT=2 like FT8, so ~10% of the symbol = ~3.2 ms
  gTranSteps         = 64;            // MUST BE 32*n
  gTranMicrosPerStep = 64;           // 32 x 100 us = 3.2 ms, 64 x 64 us = 4.096 ms, 64 x 50 us = 3.2 ms
  gTranInterpFactor  = gTranSteps / 32;

  clearQueue();                       // RTTY must not use the shared buffer
  jttyReset();
}


// ---------------------------------------------------------------------------
// Transmit (called from DigitalTXmsg when TxMode == MODE_JTTY)
// Runs until all symbols (including frames appended during TX) are sent, or
// DigiTxOn goes false ({X0}, SWR/temperature lockout).
// ---------------------------------------------------------------------------

void transmitJtty()
{
  uint32_t tStart  = millis();
  uint32_t lastFTW = 0;
  // start the transition half its length before each symbol boundary (as FTx)
  uint32_t halfTranMs = (gTranSteps * gTranMicrosPerStep) / 2000UL;

  if (jttyTune)                          // {JT}: steady carrier on tone 0 until {X0}
  {
    dds.setFTW(dds.frequencyFTW((CurFreq + FTxTxDF) * 10UL), PowerUp);
    while (DigiTxOn)                     // {X0}, SWR/temperature lockout
    {
      SWRmeasure();
      ProcessSerialIn();
    }
    goto done;
  }

  for (uint16_t i = 0; i < jttyTotalSymbols(); i++)          // re-read: chaining
  {
    uint32_t baseFTW = dds.frequencyFTW((CurFreq + FTxTxDF) * 10UL);
    uint32_t nextFTW = baseFTW +
        (((uint32_t)jttyTone(i) * tone_spacing_FTWincr_x100) + 50) / 100;

    if (i == 0) dds.setFTW(nextFTW, PowerUp);
    else        gaussianTransition(lastFTW, nextFTW);       // refactored from DigitalTXmsg
    lastFTW = nextFTW;

    // absolute schedule from tStart - no accumulated error
    uint32_t toneEnd = (uint32_t)(i + 1) * tone_duration;
    if (i + 1 < jttyTotalSymbols()) toneEnd -= halfTranMs;

    while (millis() - tStart < toneEnd)
    {
      SWRmeasure();
      ProcessSerialIn();                 // may append frames or receive {X0}
      if (!DigiTxOn) goto done;          // abort
    }
  }

done:
  dds.setFrequencyHz_x10((CurFreq + FTxTxDF) * 10, PowerDown);
  DigiTxOn = false;                      // key up (main loop handles TX hold)
  jttyReset();
}
