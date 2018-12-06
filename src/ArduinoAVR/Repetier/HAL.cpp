#include "Repetier.h"
#include "HAL.h"

#include "motion.h"

#include "temperatures.h"

#include <compat/twi.h>


#define bit_set(x,y)    x |=  (1<<y)  //sbi(x,y)
#define bit_clear(x,y)  x &= ~(1<<y)  //cbi(x,y)


HAL  hal;



#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |=  _BV(bit))
#endif




void
HAL::setup(void) {

  //
  //  Turn off the watchdog timer.
  //
  
  wdt_disable();

  //  TIMER0 uses a prescale of 64, with a 16MHz F_CPU, we'll call
  //  the EXTRUDER_INTERRUPT 250000 / 256 = 3906.25 times per second.
  //  This interrupt is used for ADVANCE mode.

  TCCR0A  = 0;
  TIMSK0 |= (1 << OCIE0A);

  //  The PDM_INTERRUPT is set to trigger every 64th (prescaled) clock,
  //  or four times as many times.

  TCCR0A  = 0;
  OCR0B   = 64;
  TIMSK0 |= (1 << OCIE0B);

  //  The STEPPER_INTERRUPT has no prescaling, and is called (initially?)
  //  244.275 times per second.

  TCCR1A  = 0;
  TCCR1C  = 0;
  TIMSK1  = 0;

  TCCR1B  = (_BV(WGM12) | _BV(CS10));   // no prescaler == 0.0625 usec tick | 001 = clk/1
  OCR1A   = 65500;                      // start off with a slow frequency.
  TIMSK1 |= (1 << OCIE1A);              // Enable interrupt
}




void(* resetFunc) (void) = 0; //declare reset function @ address 0

void HAL::resetHardware() {
  resetFunc();
}


#define TENTH_OF_SECOND  (1 * F_CPU / 40960)
#define HALF_OF_SECOND   (5 * F_CPU / 40960)

//  Update pwm values for heater.
//  Update pwm values for fans and handle kickstarting them.
//  Update a 100ms and 500ms trigger.
//  Call UI fastAction().
//
//  Called 3906.25 times per second.
//  Called once every 256 microseconds.
//  
ISR(TIMER0_COMPB_vect) {

  OCR0B += 64;   //  Reset the timer to trigger 256 microseconds from now.

  //  Count how many times we've been called.

  hal.counter100ms++;
  hal.counter500ms++;

  //  Every millisecond, do a fastAction for the user interface.

  if ((hal.counter100ms & 3) == 0x03) {
    uid.fastAction();
  }

  //  Every 4 milliseconds, update extruder temperature.

  if ((hal.counter100ms & 0x03) == 0x03) {
    extruderTemp.heaterPDM();
  }

  //  Every 16 milliseconds, update bed temperature.

  if ((hal.counter100ms & 0x07) == 0x07) {
    bedTemp.heaterPDM();
  }

  //  Every 32 milliseconds, update fan speeds.
  //  This needs to run quite fast, otherwise, the fan will pulse.

  if ((hal.counter100ms & 0x0f) == 0x0f) {
    extruderTemp.fanPDM();
    extruderTemp.fanKickTick();

    layerFan.fanPDM();
    layerFan.fanKickTick();

    //bedTemp.fanPDM();        //  No fan on the bed!
    //bedTemp.fanKickTick();
  }

  //  Reset the counter and announce it's been 100ms.

  if (hal.counter100ms >= TENTH_OF_SECOND) {
    hal.counter100ms = 0;
    hal.execute100ms = 1;
  }

  //  Reset the counter and announce it's been 500ms.

  if (hal.counter500ms >= HALF_OF_SECOND) {
    hal.counter500ms = 0;
    hal.execute500ms = 1;
  }

  //if (hal.wdPinged) {
  //  wdt_reset();
  //  hal.wdPinged = false;
  //}
}





//  Timer routine for extruder stepper.
//
//    Several methods need to move the extruder. To get a optima result,
//    all methods update the printer_state.extruderStepsNeeded with the
//    number of additional steps needed. During this interrupt, one step
//    is executed. This will keep the extruder moving, until the total
//    wanted movement is achieved. This will be done with the maximum
//    allowable speed for the extruder.
//

#define ADVANCE_DIR_FILTER_STEPS 2

int8_t extruderLastDirection = 0;

void
resetExtruderDirection() {
  extruderLastDirection = 0;
}


ISR(TIMER0_COMPA_vect) {
  uint8_t timer = OCR0A;

  if (Printer::isAdvanceActivated() == false)
    return; // currently no need

  if (Printer::extruderStepsNeeded > 0 && extruderLastDirection != 1) {
    if (Printer::extruderStepsNeeded >= ADVANCE_DIR_FILTER_STEPS) {
      extruder.setDirection(true);
      extruderLastDirection = 1;
      timer += 40; // Add some more wait time to prevent blocking
    }
  } else if(Printer::extruderStepsNeeded < 0 && extruderLastDirection != -1) {
    if(-Printer::extruderStepsNeeded >= ADVANCE_DIR_FILTER_STEPS) {
      extruder.setDirection(false);
      extruderLastDirection = -1;
      timer += 40; // Add some more wait time to prevent blocking
    }
  } else if(Printer::extruderStepsNeeded != 0) {
    extruder.step();
    Printer::extruderStepsNeeded -= extruderLastDirection;
    Printer::insertStepperHighDelay();
    extruder.unstep();
  }

  OCR0A = timer + Printer::maxExtruderSpeed;
}












// ================== Interrupt handling ======================

long __attribute__((used)) stepperWait = 0;

/** \brief Sets the timer 1 compare value to delay ticks.

    This function sets the OCR1A compare counter  to get the next interrupt
    at delay ticks measured from the last interrupt. delay must be << 2^24
*/

void
setTimer(uint32_t delay) {
  __asm__ __volatile__ (
                        "cli \n\t"
                        "tst %C[delay] \n\t" //if(delay<65536) {
                        "brne else%= \n\t"
                        "cpi %B[delay],255 \n\t"
                        "breq else%= \n\t" // delay <65280
                        "sts stepperWait,r1 \n\t" // stepperWait = 0;
                        "sts stepperWait+1,r1 \n\t"
                        "sts stepperWait+2,r1 \n\t"
                        "lds %C[delay],%[time] \n\t" // Read TCNT1
                        "lds %D[delay],%[time]+1 \n\t"
                        "ldi r18,100 \n\t" // Add 100 to TCNT1
                        "add %C[delay],r18 \n\t"
                        "adc %D[delay],r1 \n\t"
                        "cp %A[delay],%C[delay] \n\t" // delay<TCNT1+1
                        "cpc %B[delay],%D[delay] \n\t"
                        "brcc exact%= \n\t"
                        "sts %[ocr]+1,%D[delay] \n\t" //  OCR1A = TCNT1+100;
                        "sts %[ocr],%C[delay] \n\t"
                        "rjmp end%= \n\t"
                        "exact%=: sts %[ocr]+1,%B[delay] \n\t" //  OCR1A = delay;
                        "sts %[ocr],%A[delay] \n\t"
                        "rjmp end%= \n\t"
                        "else%=: subi	%B[delay], 0x80 \n\t" //} else { stepperWait = delay-32768;
                        "sbci	%C[delay], 0x00 \n\t"
                        "sts stepperWait,%A[delay] \n\t"
                        "sts stepperWait+1,%B[delay] \n\t"
                        "sts stepperWait+2,%C[delay] \n\t"
                        "ldi	%D[delay], 0x80 \n\t" //OCR1A = 32768;
                        "sts	%[ocr]+1, %D[delay] \n\t"
                        "sts	%[ocr], r1 \n\t"
                        "end%=: \n\t"
                        //:[delay]"=&d"(delay),[stepperWait]"=&d"(stepperWait) // Output
                        :[delay]"=&d"(delay) // Output
                        :"0"(delay), [ocr]"i" (_SFR_MEM_ADDR(OCR1A)), [time]"i"(_SFR_MEM_ADDR(TCNT1)) // Input
                        :"r18" // Clobber
                        );
  /* // Assembler above replaced this code
     if(delay<65280) {
     stepperWait = 0;
     unsigned int count = TCNT1+100;
     if(delay<count)
     OCR1A = count;
     else
     OCR1A = delay;
     } else {
     stepperWait = delay-32768;
     OCR1A = 32768;
     }*/
}



// volatile uint8_t insideTimer1 = 0;
/** \brief Timer interrupt routine to drive the stepper motors.
 */
ISR(TIMER1_COMPA_vect) {
  // if(insideTimer1) return;
  uint8_t doExit;
  __asm__ __volatile__ (
                        "ldi %[ex],0 \n\t"
                        "lds r23,stepperWait+2 \n\t"
                        "tst r23 \n\t" //if(stepperWait<65536) {
                        "brne else%= \n\t" // Still > 65535
                        "lds r23,stepperWait+1 \n\t"
                        "tst r23 \n\t"
                        "brne last%= \n\t" // Still not 0, go ahead
                        "lds r22,stepperWait \n\t"
                        "breq end%= \n\t" // stepperWait is 0, do your work
                        "last%=: \n\t"
                        "sts %[ocr]+1,r23 \n\t" //  OCR1A = stepper wait;
                        "sts %[ocr],r22 \n\t"
                        "sts stepperWait,r1 \n\t"
                        "sts stepperWait+1,r1 \n\t"
                        "rjmp end1%= \n\t"
                        "else%=: lds r22,stepperWait+1 \n\t" //} else { stepperWait = stepperWait-32768;
                        "subi	r22, 0x80 \n\t"
                        "sbci	r23, 0x00 \n\t"
                        "sts stepperWait+1,r22 \n\t"    // ocr1a stays 32768
                        "sts stepperWait+2,r23 \n\t"
                        "end1%=: ldi %[ex],1 \n\t"
                        "end%=: \n\t"
                        :[ex]"=&d"(doExit):[ocr]"i" (_SFR_MEM_ADDR(OCR1A)):"r22", "r23" );
  //        :[ex]"=&d"(doExit),[stepperWait]"=&d"(stepperWait):[ocr]"i" (_SFR_MEM_ADDR(OCR1A)):"r22","r23" );
  if(doExit) return;

  cbi(TIMSK1, OCIE1A); // prevent retrigger timer by disabling timer interrupt. Should be faster the guarding with insideTimer1.

  // insideTimer1 = 1;
  OCR1A = 61000;
  if(PrintLine::hasLines()) {
    setTimer(PrintLine::bresenhamStep());
  }
#if FEATURE_BABYSTEPPING
  else if(Printer::zBabystepsMissing) {
    Printer::zBabystep();
    setTimer(Printer::interval);
  }
#endif
  else {
    if(waitRelax == 0) {
      if(Printer::advanceStepsSet) {
        Printer::extruderStepsNeeded -= Printer::advanceStepsSet;
        Printer::advanceExecuted = 0;
        Printer::advanceStepsSet = 0;
      }

    } else waitRelax--;
    stepperWait = 0; // Important because of optimization in asm at begin
    OCR1A = 65500; // Wait for next move
  }

  sbi(TIMSK1, OCIE1A);
  //insideTimer1 = 0;
}

