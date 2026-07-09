/*
  avr-gcc -mmcu=attiny85 -DF_CPU=128000UL -Os -flto -ffunction-sections -fdata-sections -std=gnu11 -Wall -Wextra -Wl,--gc-sections -o motion_detector.elf motion_detector.c
  avr-objcopy -O ihex -R .eeprom motion_detector.elf motion_detector.hex
  avrdude -c stk500v1 -P /dev/ttyUSB0 -b 19200 -p t85 -U lfuse:w:0xE4:m -U hfuse:w:0xDF:m -U efuse:w:0xFF:m
  avrdude -c stk500v1 -P /dev/ttyUSB0 -b 19200 -p t85 -U flash:w:motion_detector.hex:i
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#define SYSTEM_CLOCK_HZ 128000UL

#if defined(WDTCSR)
  #define WDT_REG WDTCSR
#else
  #define WDT_REG WDTCR
#endif

#define MOTION_PIN PB2
#define MOTION_INT PCINT2

#define BUZZER_A PB0
#define BUZZER_B PB1
#define BUZZER_MASK (_BV(BUZZER_A) | _BV(BUZZER_B))

#define TIMER0_PRESCALER 64UL
#define TIMER0_CLOCK_HZ (SYSTEM_CLOCK_HZ / TIMER0_PRESCALER)
#define TIMER0_CLOCK_BITS (_BV(CS01) | _BV(CS00))
#define TIMER0_TICKS_PER_MS (TIMER0_CLOCK_HZ / 1000UL)
#define TIMER0_MAX_PERIOD_MS ((256UL * 1000UL) / TIMER0_CLOCK_HZ)

_Static_assert(
  TIMER0_CLOCK_HZ % 1000UL == 0,
  "Timer0 clock must represent whole ticks per millisecond"
);

#define TIMER1_CLOCK_HZ SYSTEM_CLOCK_HZ
#define TIMER1_CLOCK_BITS (_BV(CS10))

#define WDT_16MS 0
#define WDT_32MS _BV(WDP0)
#define WDT_64MS _BV(WDP1)
#define WDT_125MS (_BV(WDP1) | _BV(WDP0))
#define WDT_250MS _BV(WDP2)
#define WDT_500MS (_BV(WDP2) | _BV(WDP0))
#define WDT_1S (_BV(WDP2) | _BV(WDP1))
#define WDT_2S (_BV(WDP2) | _BV(WDP1) | _BV(WDP0))
#define WDT_4S _BV(WDP3)
#define WDT_8S (_BV(WDP3) | _BV(WDP0))

static volatile uint8_t MotionEvent = 0;

EMPTY_INTERRUPT(WDT_vect);
EMPTY_INTERRUPT(TIMER0_COMPA_vect);

ISR(PCINT0_vect) {
  if (PINB & _BV(MOTION_PIN)) {
    MotionEvent = 1;
  }
}

static void BuzzerFloat(void) {
  TCCR1 = 0;
  GTCCR = 0;
  TCNT1 = 0;
  PORTB = 0;
  DDRB = 0;

  #if defined(PRR) && defined(PRTIM1)
    PRR |= _BV(PRTIM1);
  #endif
}

static void ConfigurePinsForLowPower(void) {
  PORTB = 0;
  DDRB = 0;
  MCUCR |= _BV(PUD);

  #if defined(DIDR0)
    DIDR0 = 0;
    #if defined(ADC0D)
      DIDR0 |= _BV(ADC0D);
    #endif
    #if defined(ADC2D)
      DIDR0 |= _BV(ADC2D);
    #endif
    #if defined(ADC3D)
      DIDR0 |= _BV(ADC3D);
    #endif
    #if defined(AIN0D)
      DIDR0 |= _BV(AIN0D);
    #endif
    #if defined(AIN1D)
      DIDR0 |= _BV(AIN1D);
    #endif
  #endif
}

static void DisableUnusedPeripherals(void) {
  #if defined(ADCSRA) && defined(ADEN)
    ADCSRA &= ~_BV(ADEN);
  #endif

  #if defined(ACSR) && defined(ACD)
    ACSR |= _BV(ACD);
  #endif

  #if defined(PLLCSR)
    PLLCSR = 0;
  #endif

  #if defined(TIMSK)
    TIMSK = 0;
  #endif

  #if defined(TCCR0A)
    TCCR0A = 0;
  #endif

  #if defined(TCCR0B)
    TCCR0B = 0;
  #endif

  #if defined(TCCR1)
    TCCR1 = 0;
  #endif

  #if defined(GTCCR)
    GTCCR = 0;
  #endif

  #if defined(GIMSK)
    GIMSK = 0;
  #endif

  #if defined(PCMSK)
    PCMSK = 0;
  #endif

  #if defined(PRR)
    #if defined(PRADC)
      PRR |= _BV(PRADC);
    #endif
    #if defined(PRUSI)
      PRR |= _BV(PRUSI);
    #endif
    #if defined(PRTIM0)
      PRR |= _BV(PRTIM0);
    #endif
    #if defined(PRTIM1)
      PRR |= _BV(PRTIM1);
    #endif
  #endif
}

static void PrepareSleepPowerDown(void) {
  MCUCR &= ~(_BV(SM1) | _BV(SM0));
  MCUCR |= _BV(SM1);
  MCUCR |= _BV(SE);
}

static void PrepareSleepIdle(void) {
  MCUCR &= ~(_BV(SM1) | _BV(SM0));
  MCUCR |= _BV(SE);
}

static void SetWatchdogInterrupt(uint8_t periodBits) {
  wdt_reset();

  #if defined(MCUSR) && defined(WDRF)
    MCUSR &= ~_BV(WDRF);
  #endif

  WDT_REG |= _BV(WDCE) | _BV(WDE);
  WDT_REG = _BV(WDIE) | periodBits;
}

static void DisableWatchdog(void) {
  uint8_t oldSreg = SREG;
  cli();

  wdt_reset();

  #if defined(MCUSR) && defined(WDRF)
    MCUSR &= ~_BV(WDRF);
  #endif

  WDT_REG |= _BV(WDCE) | _BV(WDE);
  WDT_REG = 0;

  SREG = oldSreg;
}

static void SleepWdt(uint8_t periodBits) {
  uint8_t oldSreg = SREG;
  cli();

  BuzzerFloat();
  SetWatchdogInterrupt(periodBits);
  PrepareSleepPowerDown();

  sei();
  sleep_cpu();
  cli();

  MCUCR &= ~_BV(SE);
  DisableWatchdog();

  SREG = oldSreg;
}

static void SleepWdtMs(uint16_t ms) {
  while (ms >= 8000) {
    SleepWdt(WDT_8S);
    ms -= 8000;
  }

  while (ms >= 4000) {
    SleepWdt(WDT_4S);
    ms -= 4000;
  }

  while (ms >= 2000) {
    SleepWdt(WDT_2S);
    ms -= 2000;
  }

  while (ms >= 1000) {
    SleepWdt(WDT_1S);
    ms -= 1000;
  }

  while (ms >= 500) {
    SleepWdt(WDT_500MS);
    ms -= 500;
  }

  while (ms >= 250) {
    SleepWdt(WDT_250MS);
    ms -= 250;
  }

  while (ms >= 125) {
    SleepWdt(WDT_125MS);
    ms -= 125;
  }

  while (ms >= 64) {
    SleepWdt(WDT_64MS);
    ms -= 64;
  }

  while (ms >= 32) {
    SleepWdt(WDT_32MS);
    ms -= 32;
  }

  while (ms >= 16) {
    SleepWdt(WDT_16MS);
    ms -= 16;
  }
}

static void SleepTimer(uint8_t compareValue) {
  uint8_t oldSreg = SREG;
  cli();

  #if defined(PRR) && defined(PRTIM0)
    PRR &= ~_BV(PRTIM0);
  #endif

  TCCR0A = 0;
  TCCR0B = 0;
  TCNT0 = 0;
  OCR0A = compareValue;
  TIFR = _BV(OCF0A);
  TIMSK |= _BV(OCIE0A);
  TCCR0A = _BV(WGM01);
  TCCR0B = TIMER0_CLOCK_BITS;

  PrepareSleepIdle();

  sei();
  sleep_cpu();
  cli();

  TCCR0B = 0;
  TCCR0A = 0;
  TIMSK &= ~_BV(OCIE0A);
  TIFR = _BV(OCF0A);
  MCUCR &= ~_BV(SE);

  #if defined(PRR) && defined(PRTIM0)
    PRR |= _BV(PRTIM0);
  #endif

  SREG = oldSreg;
}

static void SleepTimerMs(uint16_t ms) {
  while (ms) {
    uint16_t chunk = ms;

    if (chunk > TIMER0_MAX_PERIOD_MS) {
      chunk = TIMER0_MAX_PERIOD_MS;
    }

    uint16_t ticks = chunk * TIMER0_TICKS_PER_MS;

    SleepTimer((uint8_t)(ticks - 1));

    ms -= chunk;
  }
}

static void EnableMotionDetection(void) {
  uint8_t oldSreg = SREG;
  cli();

  DDRB &= ~_BV(MOTION_PIN);
  PORTB &= ~_BV(MOTION_PIN);

  GIMSK &= ~_BV(PCIE);
  PCMSK |= _BV(MOTION_INT);

  MotionEvent = 0;

  GIMSK |= _BV(PCIE);

  SREG = oldSreg;
}

static void WaitForMotionEvent(void) {
  uint8_t oldSreg = SREG;
  cli();

  DisableWatchdog();
  BuzzerFloat();

  while (!MotionEvent) {
    PrepareSleepPowerDown();

    sei();
    sleep_cpu();
    cli();

    MCUCR &= ~_BV(SE);
  }

  MotionEvent = 0;

  GIMSK &= ~_BV(PCIE);
  PCMSK &= ~_BV(MOTION_INT);
  MCUCR &= ~_BV(SE);

  SREG = oldSreg;
}

static void StartToneTimer1(uint8_t top, uint8_t duty) {
  uint8_t oldSreg = SREG;
  cli();

  #if defined(PRR) && defined(PRTIM1)
    PRR &= ~_BV(PRTIM1);
  #endif

  PORTB = 0;
  DDRB |= BUZZER_MASK;
  TCCR1 = 0;
  GTCCR = 0;
  TCNT1 = 0;
  OCR1C = top;
  OCR1A = duty;

  #if defined(DT1A)
    DT1A = 0;
  #endif

  #if defined(DTPS1)
    DTPS1 = 0;
  #endif

  TIFR = _BV(TOV1) | _BV(OCF1A);
  GTCCR = _BV(PSR1);
  TCCR1 = _BV(PWM1A) | _BV(COM1A0) | TIMER1_CLOCK_BITS;

  SREG = oldSreg;
}

static void UpdateToneTimer1(uint8_t top, uint8_t duty) {
  uint8_t oldSreg = SREG;
  cli();

  TCCR1 = 0;
  TCNT1 = 0;
  OCR1C = top;
  OCR1A = duty;
  TCCR1 = _BV(PWM1A) | _BV(COM1A0) | TIMER1_CLOCK_BITS;

  SREG = oldSreg;
}

static void StopToneTimer1(void) {
  uint8_t oldSreg = SREG;
  cli();

  TCCR1 = 0;
  GTCCR = 0;
  TCNT1 = 0;
  PORTB = 0;
  DDRB = 0;

  #if defined(PRR) && defined(PRTIM1)
    PRR |= _BV(PRTIM1);
  #endif

  SREG = oldSreg;
}

static uint8_t Timer1Top(uint16_t hz) {
  uint32_t period = (TIMER1_CLOCK_HZ + (hz / 2)) / hz;

  if (period < 4) {
    period = 4;
  }

  if (period > 256) {
    period = 256;
  }

  return (uint8_t)(period - 1);
}

static uint8_t Timer1Duty(uint8_t top) {
  uint16_t period = (uint16_t)top + 1;
  return (uint8_t)(period / 2);
}

static void Tone(uint16_t hz, uint16_t ms) {
  uint8_t top = Timer1Top(hz);
  uint8_t duty = Timer1Duty(top);

  StartToneTimer1(top, duty);
  SleepTimerMs(ms);
  StopToneTimer1();
}

static void ToneSweep(uint16_t fromHz, uint16_t toHz, uint16_t ms) {
  const uint8_t steps = 16;
  uint16_t stepMs = ms / steps;
  uint8_t extraMs = ms % steps;
  int32_t difference = (int32_t)toHz - (int32_t)fromHz;

  uint8_t top = Timer1Top(fromHz);
  StartToneTimer1(top, Timer1Duty(top));

  for (uint8_t i = 0; i < steps; i++) {
    if (i > 0) {
      uint16_t hz =
          (uint16_t)((int32_t)fromHz + (difference * i) / (steps - 1));

      top = Timer1Top(hz);
      UpdateToneTimer1(top, Timer1Duty(top));
    }

    SleepTimerMs(stepMs + (i < extraMs));
  }

  StopToneTimer1();
}

static void RingtonePart(void) {
  ToneSweep(3400, 4300, 90);
  Tone(4400, 55);
  SleepWdtMs(32);

  ToneSweep(4300, 3700, 80);
  Tone(4050, 65);
  SleepWdtMs(32);

  Tone(3650, 75);
  SleepWdtMs(32);

  Tone(4100, 75);
  SleepWdtMs(32);

  Tone(4550, 110);
}

static void Ringtone(void) {
  RingtonePart();
  SleepWdtMs(125);

  RingtonePart();
  SleepWdtMs(125);

  RingtonePart();
  SleepWdtMs(750);
}

int main(void) {
  ConfigurePinsForLowPower();
  DisableUnusedPeripherals();
  DisableWatchdog();

  while (1) {
    EnableMotionDetection();
    WaitForMotionEvent();
    Ringtone();
  }
}
