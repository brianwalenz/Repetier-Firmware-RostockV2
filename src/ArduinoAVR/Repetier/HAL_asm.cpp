#include "Repetier.h"
#include "HAL.h"

#include "motion.h"

#include "temperatures.h"

#include <compat/twi.h>



const
uint16_t
fast_div_lut[17] PROGMEM = { 0,
                             F_CPU / 4096,
                             F_CPU / 8192,
                             F_CPU / 12288,
                             F_CPU / 16384,
                             F_CPU / 20480,
                             F_CPU / 24576,
                             F_CPU / 28672,
                             F_CPU / 32768,
                             F_CPU / 36864,
                             F_CPU / 40960,
                             F_CPU / 45056,
                             F_CPU / 49152,
                             F_CPU / 53248,
                             F_CPU / 57344,
                             F_CPU / 61440,
                             F_CPU / 65536 };

const
uint16_t
slow_div_lut[257] PROGMEM = { 0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              F_CPU / 256,
                              F_CPU / 288,
                              F_CPU / 320,
                              F_CPU / 352,
                              F_CPU / 384,
                              F_CPU / 416,
                              F_CPU / 448,
                              F_CPU / 480,
                              F_CPU / 512,
                              F_CPU / 544,
                              F_CPU / 576,
                              F_CPU / 608,
                              F_CPU / 640,
                              F_CPU / 672,
                              F_CPU / 704,
                              F_CPU / 736,
                              F_CPU / 768,
                              F_CPU / 800,
                              F_CPU / 832,
                              F_CPU / 864,
                              F_CPU / 896,
                              F_CPU / 928,
                              F_CPU / 960,
                              F_CPU / 992,
                              F_CPU / 1024,
                              F_CPU / 1056,
                              F_CPU / 1088,
                              F_CPU / 1120,
                              F_CPU / 1152,
                              F_CPU / 1184,
                              F_CPU / 1216,
                              F_CPU / 1248,
                              F_CPU / 1280,
                              F_CPU / 1312,
                              F_CPU / 1344,
                              F_CPU / 1376,
                              F_CPU / 1408,
                              F_CPU / 1440,
                              F_CPU / 1472,
                              F_CPU / 1504,
                              F_CPU / 1536,
                              F_CPU / 1568,
                              F_CPU / 1600,
                              F_CPU / 1632,
                              F_CPU / 1664,
                              F_CPU / 1696,
                              F_CPU / 1728,
                              F_CPU / 1760,
                              F_CPU / 1792,
                              F_CPU / 1824,
                              F_CPU / 1856,
                              F_CPU / 1888,
                              F_CPU / 1920,
                              F_CPU / 1952,
                              F_CPU / 1984,
                              F_CPU / 2016,
                              F_CPU / 2048,
                              F_CPU / 2080,
                              F_CPU / 2112,
                              F_CPU / 2144,
                              F_CPU / 2176,
                              F_CPU / 2208,
                              F_CPU / 2240,
                              F_CPU / 2272,
                              F_CPU / 2304,
                              F_CPU / 2336,
                              F_CPU / 2368,
                              F_CPU / 2400,
                              F_CPU / 2432,
                              F_CPU / 2464,
                              F_CPU / 2496,
                              F_CPU / 2528,
                              F_CPU / 2560,
                              F_CPU / 2592,
                              F_CPU / 2624,
                              F_CPU / 2656,
                              F_CPU / 2688,
                              F_CPU / 2720,
                              F_CPU / 2752,
                              F_CPU / 2784,
                              F_CPU / 2816,
                              F_CPU / 2848,
                              F_CPU / 2880,
                              F_CPU / 2912,
                              F_CPU / 2944,
                              F_CPU / 2976,
                              F_CPU / 3008,
                              F_CPU / 3040,
                              F_CPU / 3072,
                              F_CPU / 3104,
                              F_CPU / 3136,
                              F_CPU / 3168,
                              F_CPU / 3200,
                              F_CPU / 3232,
                              F_CPU / 3264,
                              F_CPU / 3296,
                              F_CPU / 3328,
                              F_CPU / 3360,
                              F_CPU / 3392,
                              F_CPU / 3424,
                              F_CPU / 3456,
                              F_CPU / 3488,
                              F_CPU / 3520,
                              F_CPU / 3552,
                              F_CPU / 3584,
                              F_CPU / 3616,
                              F_CPU / 3648,
                              F_CPU / 3680,
                              F_CPU / 3712,
                              F_CPU / 3744,
                              F_CPU / 3776,
                              F_CPU / 3808,
                              F_CPU / 3840,
                              F_CPU / 3872,
                              F_CPU / 3904,
                              F_CPU / 3936,
                              F_CPU / 3968,
                              F_CPU / 4000,
                              F_CPU / 4032,
                              F_CPU / 4064,
                              F_CPU / 4096,
                              F_CPU / 4128,
                              F_CPU / 4160,
                              F_CPU / 4192,
                              F_CPU / 4224,
                              F_CPU / 4256,
                              F_CPU / 4288,
                              F_CPU / 4320,
                              F_CPU / 4352,
                              F_CPU / 4384,
                              F_CPU / 4416,
                              F_CPU / 4448,
                              F_CPU / 4480,
                              F_CPU / 4512,
                              F_CPU / 4544,
                              F_CPU / 4576,
                              F_CPU / 4608,
                              F_CPU / 4640,
                              F_CPU / 4672,
                              F_CPU / 4704,
                              F_CPU / 4736,
                              F_CPU / 4768,
                              F_CPU / 4800,
                              F_CPU / 4832,
                              F_CPU / 4864,
                              F_CPU / 4896,
                              F_CPU / 4928,
                              F_CPU / 4960,
                              F_CPU / 4992,
                              F_CPU / 5024,
                              F_CPU / 5056,
                              F_CPU / 5088,
                              F_CPU / 5120,
                              F_CPU / 5152,
                              F_CPU / 5184,
                              F_CPU / 5216,
                              F_CPU / 5248,
                              F_CPU / 5280,
                              F_CPU / 5312,
                              F_CPU / 5344,
                              F_CPU / 5376,
                              F_CPU / 5408,
                              F_CPU / 5440,
                              F_CPU / 5472,
                              F_CPU / 5504,
                              F_CPU / 5536,
                              F_CPU / 5568,
                              F_CPU / 5600,
                              F_CPU / 5632,
                              F_CPU / 5664,
                              F_CPU / 5696,
                              F_CPU / 5728,
                              F_CPU / 5760,
                              F_CPU / 5792,
                              F_CPU / 5824,
                              F_CPU / 5856,
                              F_CPU / 5888,
                              F_CPU / 5920,
                              F_CPU / 5952,
                              F_CPU / 5984,
                              F_CPU / 6016,
                              F_CPU / 6048,
                              F_CPU / 6080,
                              F_CPU / 6112,
                              F_CPU / 6144,
                              F_CPU / 6176,
                              F_CPU / 6208,
                              F_CPU / 6240,
                              F_CPU / 6272,
                              F_CPU / 6304,
                              F_CPU / 6336,
                              F_CPU / 6368,
                              F_CPU / 6400,
                              F_CPU / 6432,
                              F_CPU / 6464,
                              F_CPU / 6496,
                              F_CPU / 6528,
                              F_CPU / 6560,
                              F_CPU / 6592,
                              F_CPU / 6624,
                              F_CPU / 6656,
                              F_CPU / 6688,
                              F_CPU / 6720,
                              F_CPU / 6752,
                              F_CPU / 6784,
                              F_CPU / 6816,
                              F_CPU / 6848,
                              F_CPU / 6880,
                              F_CPU / 6912,
                              F_CPU / 6944,
                              F_CPU / 6976,
                              F_CPU / 7008,
                              F_CPU / 7040,
                              F_CPU / 7072,
                              F_CPU / 7104,
                              F_CPU / 7136,
                              F_CPU / 7168,
                              F_CPU / 7200,
                              F_CPU / 7232,
                              F_CPU / 7264,
                              F_CPU / 7296,
                              F_CPU / 7328,
                              F_CPU / 7360,
                              F_CPU / 7392,
                              F_CPU / 7424,
                              F_CPU / 7456,
                              F_CPU / 7488,
                              F_CPU / 7520,
                              F_CPU / 7552,
                              F_CPU / 7584,
                              F_CPU / 7616,
                              F_CPU / 7648,
                              F_CPU / 7680,
                              F_CPU / 7712,
                              F_CPU / 7744,
                              F_CPU / 7776,
                              F_CPU / 7808,
                              F_CPU / 7840,
                              F_CPU / 7872,
                              F_CPU / 7904,
                              F_CPU / 7936,
                              F_CPU / 7968,
                              F_CPU / 8000,
                              F_CPU / 8032,
                              F_CPU / 8064,
                              F_CPU / 8096,
                              F_CPU / 8128,
                              F_CPU / 8160,
                              F_CPU / 8192  };






// http://www.mikrocontroller.net/articles/AVR_Arithmetik#32_Bit_.2F_32_Bit
//
// Fast and short 32 bits AVR sqrt routine, avr-gcc ABI compliant
// R25:R24 = SQRT (R25:R24:R23:R22) rounded to the
// nearest integer (0.5 rounds up)
// Destroys R18-R19,R22-R23,R26-R27
// Cycles incl call & ret = 265-310
// Stack incl call = 2-3
//
uint16_t
integerSqrt(uint32_t a) {
  uint16_t b;

  __asm__ __volatile__ (
                        "ldi   R19, 0xc0 \n\t"
                        "clr   R18 \n\t"        // rotation mask in R19:R18
                        "ldi   R27, 0x40 \n\t"
                        "sub   R26, R26 \n\t"   // developing sqrt in R27:R26, C=0
                        "1:  brcs  2f \n\t"           // C --> Bit is always 1
                        "cp    %C1, R26 \n\t"
                        "cpc   %D1, R27 \n\t"     // Does test value fit?
                        "brcs  3f \n\t"           // C --> nope, bit is 0
                        "2:  sub   %C1, R26 \n\t"
                        "sbc   %D1, R27 \n\t"     // Adjust argument for next bit
                        "or    R26, R18 \n\t"
                        "or    R27, R19 \n\t"     // Set bit to 1
                        "3:  lsr   R19 \n\t"
                        "ror   R18 \n\t"          // Shift right mask, C --> end loop
                        "eor   R27, R19 \n\t"
                        "eor   R26, R18 \n\t"     // Shift right only test bit in result
                        "rol   %A1 \n\t"          // Bit 0 only set if end of loop
                        "rol   %B1 \n\t"
                        "rol   %C1 \n\t"
                        "rol   %D1 \n\t"          // Shift left remaining argument (C used at 1:)
                        "sbrs  %A1, 0 \n\t"       // Skip if 15 bits developed
                        "rjmp  1b \n\t"           // Develop 15 bits of the sqrt
                        "brcs  4f \n\t"           // C--> Last bits always 1
                        "cp    R26, %C1 \n\t"
                        "cpc   R27, %D1 \n\t"     // Test for last bit 1
                        "brcc  5f \n\t"           // NC --> bit is 0
                        "4:  sbc   %B1, R19 \n\t"     // Subtract C (any value from 1 to 0x7f will do)
                        "sbc   %C1, R26 \n\t"
                        "sbc   %D1, R27 \n\t"     // Update argument for test
                        "inc   R26 \n\t"          // Last bit is 1
                        "5:  lsl   %B1 \n\t"          // Only bit 7 matters
                        "rol   %C1 \n\t"
                        "rol   %D1 \n\t"          // Remainder * 2 + C
                        "brcs  6f \n\t"           // C --> Always round up
                        "cp    R26, %C1 \n\t"
                        "cpc   R27, %D1 \n\t"     // C decides rounding
                        "6:  adc   R26, R19 \n\t"
                        "adc   R27, R19 \n\t"     // Round up if C (R19=0)
                        "mov   %B0, R27 \n\t"     // return in R25:R24 for avr-gcc ABI compliance
                        "mov   %A0, R26 \n\t"
                        :"=r"(b)
                        :"r"(a)
                        :"r18", "r19", "r27", "r26" );

  return b;
}




//  Optimized division
//
//  Normally the C compiler will compute a long/long division, which takes ~670 Ticks.
//  This version is optimized for a 16 bit dividend and recognizes the special cases
//  of a 24 bit and 16 bit dividend, which often, but not always occur in updating the
//  interval.
//
int32_t
Div4U2U(uint32_t a,uint16_t b) {
  // r14/r15 remainder
  // r16 counter
  __asm__ __volatile__ (
                        "clr r14 \n\t"
                        "sub r15,r15 \n\t"
                        "tst %D0 \n\t"
                        "brne do32%= \n\t"
                        "tst %C0 \n\t"
                        "breq donot24%= \n\t"
                        "rjmp do24%= \n\t"
                        "donot24%=:" "ldi r16,17 \n\t" // 16 Bit divide
                        "d16u_1%=:" "rol %A0 \n\t"
                        "rol %B0 \n\t"
                        "dec r16 \n\t"
                        "brne	d16u_2%= \n\t"
                        "rjmp end%= \n\t"
                        "d16u_2%=:" "rol r14 \n\t"
                        "rol r15 \n\t"
                        "sub r14,%A2 \n\t"
                        "sbc r15,%B2 \n\t"
                        "brcc	d16u_3%= \n\t"
                        "add r14,%A2 \n\t"
                        "adc r15,%B2 \n\t"
                        "clc \n\t"
                        "rjmp d16u_1%= \n\t"
                        "d16u_3%=:" "sec \n\t"
                        "rjmp d16u_1%= \n\t"
                        "do32%=:" // divide full 32 bit
                        "rjmp do32B%= \n\t"
                        "do24%=:" // divide 24 bit

                        "ldi r16,25 \n\t" // 24 Bit divide
                        "d24u_1%=:" "rol %A0 \n\t"
                        "rol %B0 \n\t"
                        "rol %C0 \n\t"
                        "dec r16 \n\t"
                        "brne	d24u_2%= \n\t"
                        "rjmp end%= \n\t"
                        "d24u_2%=:" "rol r14 \n\t"
                        "rol r15 \n\t"
                        "sub r14,%A2 \n\t"
                        "sbc r15,%B2 \n\t"
                        "brcc	d24u_3%= \n\t"
                        "add r14,%A2 \n\t"
                        "adc r15,%B2 \n\t"
                        "clc \n\t"
                        "rjmp d24u_1%= \n\t"
                        "d24u_3%=:" "sec \n\t"
                        "rjmp d24u_1%= \n\t"

                        "do32B%=:" // divide full 32 bit

                        "ldi r16,33 \n\t" // 32 Bit divide
                        "d32u_1%=:" "rol %A0 \n\t"
                        "rol %B0 \n\t"
                        "rol %C0 \n\t"
                        "rol %D0 \n\t"
                        "dec r16 \n\t"
                        "brne	d32u_2%= \n\t"
                        "rjmp end%= \n\t"
                        "d32u_2%=:" "rol r14 \n\t"
                        "rol r15 \n\t"
                        "sub r14,%A2 \n\t"
                        "sbc r15,%B2 \n\t"
                        "brcc	d32u_3%= \n\t"
                        "add r14,%A2 \n\t"
                        "adc r15,%B2 \n\t"
                        "clc \n\t"
                        "rjmp d32u_1%= \n\t"
                        "d32u_3%=:" "sec \n\t"
                        "rjmp d32u_1%= \n\t"

                        "end%=:" // end
                        :"=&r"(a)
                        :"0"(a),"r"(b)
                        :"r14","r15","r16"
                        );
  return a;
}



unsigned long
U16SquaredToU32(unsigned int val) {
  long res;
  __asm__ __volatile__ ( // 15 Ticks
                        "mul %A1,%A1 \n\t"
                        "movw %A0,r0 \n\t"
                        "mul %B1,%B1 \n\t"
                        "movw %C0,r0 \n\t"
                        "mul %A1,%B1 \n\t"
                        "clr %A1 \n\t"
                        "add %B0,r0 \n\t"
                        "adc %C0,r1 \n\t"
                        "adc %D0,%A1 \n\t"
                        "add %B0,r0 \n\t"
                        "adc %C0,r1 \n\t"
                        "adc %D0,%A1 \n\t"
                        "clr r1 \n\t"
                        : "=&r"(res),"=r"(val)
                        : "1"(val)
                         );
  return res;
}



unsigned int
ComputeV(long timer,long accel) {
  unsigned int res;
  // 38 Ticks
  __asm__ __volatile__ ( // 0 = res, 1 = timer, 2 = accel %D2=0 ,%A1 are unused is free
                        // Result LSB first: %A0, %B0, %A1
                        "mul %B1,%A2 \n\t"
                        "mov %A0,r1 \n\t"
                        "mul %B1,%C2 \n\t"
                        "mov %B0,r0 \n\t"
                        "mov %A1,r1 \n\t"
                        "mul %B1,%B2 \n\t"
                        "add %A0,r0 \n\t"
                        "adc %B0,r1 \n\t"
                        "adc %A1,%D2 \n\t"
                        "mul %C1,%A2 \n\t"
                        "add %A0,r0 \n\t"
                        "adc %B0,r1 \n\t"
                        "adc %A1,%D2 \n\t"
                        "mul %C1,%B2 \n\t"
                        "add %B0,r0 \n\t"
                        "adc %A1,r1 \n\t"
                        "mul %D1,%A2 \n\t"
                        "add %B0,r0 \n\t"
                        "adc %A1,r1 \n\t"
                        "mul %C1,%C2 \n\t"
                        "add %A1,r0 \n\t"
                        "mul %D1,%B2 \n\t"
                        "add %A1,r0 \n\t"
                        "lsr %A1 \n\t"
                        "ror %B0 \n\t"
                        "ror %A0 \n\t"
                        "lsr %A1 \n\t"
                        "ror %B0 \n\t"
                        "ror %A0 \n\t"
                        "clr r1 \n\t"
                        :"=&r"(res),"=r"(timer),"=r"(accel)
                        :"1"(timer),"2"(accel)
                        : );
  // unsigned int v = ((timer>>8)*cur->accel)>>10;
  return res;
}


// Multiply two 16 bit values and return 32 bit result
//
uint32_t
mulu16xu16to32(unsigned int a,unsigned int b) {
  uint32_t res;
  // 18 Ticks = 1.125 us
  __asm__ __volatile__ ( // 0 = res, 1 = timer, 2 = accel %D2=0 ,%A1 are unused is free
                        // Result LSB first: %A0, %B0, %A1
                        "clr r18 \n\t"
                        "mul %B2,%B1 \n\t" // mul hig bytes
                        "movw %C0,r0 \n\t"
                        "mul %A1,%A2 \n\t" // mul low bytes
                        "movw %A0,r0 \n\t"
                        "mul %A1,%B2 \n\t"
                        "add %B0,r0 \n\t"
                        "adc %C0,r1 \n\t"
                        "adc %D0,r18 \n\t"
                        "mul %B1,%A2 \n\t"
                        "add %B0,r0 \n\t"
                        "adc %C0,r1 \n\t"
                        "adc %D0,r18 \n\t"
                        "clr r1 \n\t"
                        :"=&r"(res),"=r"(a),"=r"(b)
                        :"1"(a),"2"(b)
                        :"r18" );
  // return (long)a*b;
  return res;
}




// Multiply two 16 bit values and return 32 bit result
unsigned int
mulu6xu16shift16(unsigned int a,unsigned int b) {
  unsigned int res;
  // 18 Ticks = 1.125 us
  __asm__ __volatile__ ( // 0 = res, 1 = timer, 2 = accel %D2=0 ,%A1 are unused is free
                        // Result LSB first: %A0, %B0, %A1
                        "clr r18 \n\t"
                        "mul %B2,%B1 \n\t" // mul hig bytes
                        "movw %A0,r0 \n\t"
                        "mul %A1,%A2 \n\t" // mul low bytes
                        "mov r19,r1 \n\t"
                        "mul %A1,%B2 \n\t"
                        "add r19,r0 \n\t"
                        "adc %A0,r1 \n\t"
                        "adc %B0,r18 \n\t"
                        "mul %B1,%A2 \n\t"
                        "add r19,r0 \n\t"
                        "adc %A0,r1 \n\t"
                        "adc %B0,r18 \n\t"
                        "clr r1 \n\t"
                        :"=&r"(res),"=r"(a),"=r"(b)
                        :"1"(a),"2"(b)
                        :"r18","r19" );
  return res;
}




/** \brief approximates division of F_CPU/divisor

    In the stepper interrupt a division is needed, which is a slow operation.
    The result is used for timer calculation where small errors are ok. This
    function uses lookup tables to find a fast approximation of the result.

*/

int32_t
CPUDivU2(unsigned int divisor) {
  int32_t res;
  unsigned short table;
  if(divisor < 8192) {
    if(divisor < 512) {
      if(divisor < 10) divisor = 10;
      return Div4U2U(F_CPU, divisor); // These entries have overflows in lookuptable!
    }
    table = (unsigned short)&slow_div_lut[0];
    __asm__ __volatile__( // needs 64 ticks neu 49 Ticks
                         "mov r18,%A1 \n\t"
                         "andi r18,31 \n\t"  // divisor & 31 in r18
                         "lsr %B1 \n\t" // divisor >> 4
                         "ror %A1 \n\t"
                         "lsr %B1 \n\t"
                         "ror %A1 \n\t"
                         "lsr %B1 \n\t"
                         "ror %A1 \n\t"
                         "lsr %B1 \n\t"
                         "ror %A1 \n\t"
                         "andi %A1,254 \n\t"
                         "add %A2,%A1 \n\t" // table+divisor>>3
                         "adc %B2,%B1 \n\t"
                         "lpm %A0,Z+ \n\t" // y0 in res
                         "lpm %B0,Z+ \n\t"  // %C0,%D0 are 0
                         "movw r4,%A0 \n\t" // y0 nach gain (r4-r5)
                         "lpm r0,Z+ \n\t" // gain = gain-y1
                         "sub r4,r0 \n\t"
                         "lpm r0,Z+ \n\t"
                         "sbc r5,r0 \n\t"
                         "mul r18,r4 \n\t" // gain*(divisor & 31)
                         "movw %A1,r0 \n\t" // divisor not needed any more, use for byte 0,1 of result
                         "mul r18,r5 \n\t"
                         "add %B1,r0 \n\t"
                         "mov %A2,r1 \n\t"
                         "lsl %A1 \n\t"
                         "rol %B1 \n\t"
                         "rol %A2 \n\t"
                         "lsl %A1 \n\t"
                         "rol %B1 \n\t"
                         "rol %A2 \n\t"
                         "lsl %A1 \n\t"
                         "rol %B1 \n\t"
                         "rol %A2 \n\t"
                         "sub %A0,%B1 \n\t"
                         "sbc %B0,%A2 \n\t"
                         "clr %C0 \n\t"
                         "clr %D0 \n\t"
                         "clr r1 \n\t"
                         : "=&r" (res), "=&d"(divisor), "=&z"(table) : "1"(divisor), "2"(table) : "r18", "r4", "r5");
    return res;
    /*unsigned short adr0 = (unsigned short)&slow_div_lut+(divisor>>4)&1022;
      long y0=    pgm_read_dword_near(adr0);
      long gain = y0-pgm_read_dword_near(adr0+2);
      return y0-((gain*(divisor & 31))>>5);*/
  } else {
    table = (unsigned short)&fast_div_lut[0];
    __asm__ __volatile__( // needs 49 ticks
                         "movw r18,%A1 \n\t"
                         "andi r19,15 \n\t"  // divisor & 4095 in r18,r19
                         "lsr %B1 \n\t" // divisor >> 3, then %B1 is 2*(divisor >> 12)
                         "lsr %B1 \n\t"
                         "lsr %B1 \n\t"
                         "andi %B1,254 \n\t"
                         "add %A2,%B1 \n\t" // table+divisor>>11
                         "adc %B2,r1 \n\t" //
                         "lpm %A0,Z+ \n\t" // y0 in res
                         "lpm %B0,Z+ \n\t"
                         "movw r4,%A0 \n\t" // y0 to gain (r4-r5)
                         "lpm r0,Z+ \n\t" // gain = gain-y1
                         "sub r4,r0 \n\t"
                         "lpm r0,Z+ \n\t"
                         "sbc r5,r0 \n\t" // finished - result has max. 16 bit
                         "mul r18,r4 \n\t" // gain*(divisor & 4095)
                         "movw %A1,r0 \n\t" // divisor not needed any more, use for byte 0,1 of result
                         "mul r19,r5 \n\t"
                         "mov %A2,r0 \n\t" // %A2 = byte 3 of result
                         "mul r18,r5 \n\t"
                         "add %B1,r0 \n\t"
                         "adc %A2,r1 \n\t"
                         "mul r19,r4 \n\t"
                         "add %B1,r0 \n\t"
                         "adc %A2,r1 \n\t"
                         "andi %B1,240 \n\t" // >> 12
                         "swap %B1 \n\t"
                         "swap %A2 \r\n"
                         "mov %A1,%A2 \r\n"
                         "andi %A1,240 \r\n"
                         "or %B1,%A1 \r\n"
                         "andi %A2,15 \r\n"
                         "sub %A0,%B1 \n\t"
                         "sbc %B0,%A2 \n\t"
                         "clr %C0 \n\t"
                         "clr %D0 \n\t"
                         "clr r1 \n\t"
                         : "=&r" (res), "=&d"(divisor), "=&z"(table) : "1"(divisor), "2"(table) : "r18", "r19", "r4", "r5");
    return res;
    /*
    // The asm mimics the following code
    unsigned short adr0 = (unsigned short)&fast_div_lut+(divisor>>11)&254;
    unsigned short y0=  pgm_read_word_near(adr0);
    unsigned short gain = y0-pgm_read_word_near(adr0+2);
    return y0-(((long)gain*(divisor & 4095))>>12);*/
  }
}




