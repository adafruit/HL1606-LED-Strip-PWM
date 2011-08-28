#include "HL1606stripPWM.h"
// Written from scratch, but inspired by
// http://waitingforbigo.com/2010/10/02/second-preview-release-of-fastspi_led-library
// This library can PWM an HL1606 strip with 3 or 4 bits of color per LED
// See the example sketches for more detailed usage

// (c) Adafruit Industries / Limor Fried 2010. Released under MIT license.


static uint8_t pwmincr;
static uint8_t pwmcounter;

// the arrays of bytes that hold each LED's PWM values
static uint8_t *redPWM;
static uint8_t *greenPWM;
static uint8_t *bluePWM;

// how many LEDs
static uint8_t nLEDs;

// Brightness divider: shift all levels this many bits to the right, dimming the display
static uint8_t PWMshifter;

// the latch pin
static uint8_t latchPin;




HL1606stripPWM::HL1606stripPWM(uint8_t n, uint8_t l) {
  nLEDs = n;
  latchPin = l;

  pwmcounter = 0;
  PWMbits = 3;
  PWMshifter = 0;
  CPUmaxpercent = 70;
  pwmincr = 256 / (1 << PWMbits);
  SPIspeedDiv = 32;

  redPWM = (uint8_t *)malloc(nLEDs);
  greenPWM = (uint8_t *)malloc(nLEDs);
  bluePWM = (uint8_t *)malloc(nLEDs);
  for (uint8_t i=0; i< nLEDs; i++) {
    setLEDcolorPWM(i, 0, 0, 0);
  }
}

void HL1606stripPWM::begin(void) {
  SPIinit();
  timerinit();

  // go!
#if defined(__AVR_ATmega32U4__)
  TIMSK3 = _BV(OCIE3A);
#else
  TIMSK2 = _BV(OCIE2A);             // run our strip-writing interrupt
#endif
}


void HL1606stripPWM::timerinit(void) {
  // calculate how long it will take to pulse one strip down
  double time = nLEDs;    // each LED
  time *= 8;              // 8 bits of data for each LED;
  time *= SPIspeedDiv;    // larger dividers = more time per bit
  time *= 1000;           // time in milliseconds
  time /= F_CPU;          // multiplied by how long it takes for one instruction (nverse of cpu)

  time *= 100;            // calculate what percentage of CPU we can use (multiply by time)
  time /= CPUmaxpercent;


  // set up the interrupt to write to the entire strip
  // Each pixel requires a 1-byte SPI transfer, so with a n-pixel strip, thats n bytes at 1 MHz
  // which makes for a 1 MHz / n PWM frequency. Say for a 100 LED strip, we can update no faster
  // than 10 KHz. Lets make it 1 KHz to start

#if defined(__AVR_ATmega32U4__)
  // for the atmega32u4 we'll use counter #3
  TCCR3A = 0;
  TCCR3B = _BV(CS32) | _BV(WGM32);  // CTC mode, /256 clock
  OCR3A = (F_CPU/256) / (1000 / time);
  TCNT3 = 0;
#else
  // we'll use timer 2 in CTC mode
  TCCR2A = _BV(WGM21);     // CTC mode
  TCCR2B =  _BV(CS21) | _BV(CS22);   // 256 divider, run timer2 at 62.5 KHz (16MHz/256)
  OCR2A = (F_CPU/256) / (1000 / time);
  TCNT2 = 0;
#endif
}



void HL1606stripPWM::setLEDcolorPWM(uint8_t n, uint8_t r, uint8_t g, uint8_t b) {
     redPWM[n] = r;
     greenPWM[n] = g;
     bluePWM[n] = b;
}

void HL1606stripPWM::set(uint8_t n, uint8_t r, uint8_t g, uint8_t b) {
  this->set(n, r, g, b);
}

void HL1606stripPWM::setLEDcolorPWM(uint8_t n, uint32_t color) {
  // >> has operator precedence over &
  this->setLEDcolorPWM(n, color >> 16 & 0xFF, color >> 8 & 0xFF, color >> 0 & 0xFF);
}

void HL1606stripPWM::set(uint8_t n, uint32_t color) {
  this->setLEDcolorPWM(n, color);
}

uint32_t HL1606stripPWM::get(uint8_t n) {
  return ((uint32_t)redPWM[n] << 16 & 0xff0000) |
         ((uint32_t)greenPWM[n] << 8 & 0x00ff00) |
         ((uint32_t)bluePWM[n] << 0 & 0x0000ff);
}

#if defined(__AVR_ATmega32U4__)
ISR(TIMER3_COMPA_vect) {
#else
ISR(TIMER2_COMPA_vect) {
#endif
  uint8_t i, d;

  // write out data to strip
  for (i=0; i< nLEDs; i++) {
    d = 0x80;          // set the latch bit
    // calculate the next LED's byte
    if (pwmcounter < redPWM[i] >> PWMshifter) {
      d |= 0x04;
    }
    if (pwmcounter < greenPWM[i] >> PWMshifter) {
      d |= 0x10;
    }
    if (pwmcounter < bluePWM[i] >> PWMshifter) {
      d |= 0x01;
    }

    // check that previous xfer completed
    while(!(SPSR & _BV(SPIF)));

    // send new data
    SPDR = d;
  }

  // increment our PWM counter
  pwmcounter += pwmincr;

  // make sure we're all done
  while(!(SPSR & _BV(SPIF)));

  // latch
  digitalWrite(latchPin, HIGH);
  delayMicroseconds(3);
  digitalWrite(latchPin, LOW);
}


void HL1606stripPWM::SPIinit(void) {
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(latchPin, OUTPUT);

  // set up high speed SPI for 500 KHz
  // The datasheet says that the clock pulse width must be > 300ns. Two pulses > 600ns that would
  // make the max frequency 1.6 MHz - fat chance getting that out of HL1606's though
  SPCR = _BV(SPE) | _BV(MSTR);   // enable SPI master mode
  setSPIdivider(SPIspeedDiv);          // SPI clock is FCPU/32 = 500 Khz for most arduinos

  // we send a fake SPI byte to get the 'finished' bit set in the register, dont remove!!!
  SPDR = 0;
}

uint8_t HL1606stripPWM::getSPIdivider(void) { return SPIspeedDiv; }

void HL1606stripPWM::setSPIdivider(uint8_t spispeed) {
  SPIspeedDiv = spispeed;

  switch (spispeed) {
    case 2:
      SPSR |= _BV(SPI2X);
      break;
    case 4:
      // no bits set
      break;
    case 8:
      SPCR |= _BV(SPR0);
      SPSR |= _BV(SPI2X);
      break;
    case 16:
      SPCR |= _BV(SPR0);
      break;
    case 32:
      SPCR |= _BV(SPR1);
      SPSR |= _BV(SPI2X);
      break;
    case 64:
      SPCR |= _BV(SPR1);
      break;
    default:      // slowest
    case 128:
      SPCR |= _BV(SPR1);
      SPCR |= _BV(SPR0);
  }

  timerinit();
}

uint8_t HL1606stripPWM::numLEDs(void) {
  return nLEDs;
}

void HL1606stripPWM::setPWMbits(uint8_t b) {
  PWMbits = b;
  pwmincr = 256 / (1 << PWMbits);
}

uint8_t HL1606stripPWM::getPWMbits(void) {
  return PWMbits;
}

void HL1606stripPWM::setBrightness(uint8_t b) {
  PWMshifter = 8 - b;
}

uint8_t HL1606stripPWM::getBrightness(void) {
  return 8 - PWMshifter;
}

uint8_t HL1606stripPWM::nextBrightness(void) {
  int next = this->getBrightness() - 1;
  if (next < 0) {
    next = 8;
  } else if (next < 8 - this->getPWMbits()) {
    // Only move to the next discernable brightness level.
    next = 0;
  }
  this->setBrightness(next);
  return next;
}


uint8_t HL1606stripPWM::getCPUmax(void) {
  return CPUmaxpercent;
}

void HL1606stripPWM::setCPUmax(uint8_t cpumax) {
  CPUmaxpercent = cpumax;

  timerinit();
}
