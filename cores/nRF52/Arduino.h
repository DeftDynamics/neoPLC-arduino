#ifndef Arduino_h
#define Arduino_h

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

typedef bool boolean;
typedef uint8_t byte;
typedef uint16_t word;

// some libraries and sketches depend on this AVR stuff,
// assuming Arduino.h or WProgram.h automatically includes it...

#include "avr/pgmspace.h"
#include "avr/interrupt.h"

#include "itoa.h"

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

#include "wiring_constants.h"

#define clockCyclesPerMicrosecond() ( SystemCoreClock / 1000000L )
#define clockCyclesToMicroseconds(a) ( ((a) * 1000L) / (SystemCoreClock / 1000L) )
#define microsecondsToClockCycles(a) ( (a) * (SystemCoreClock / 1000000L) )

void yield( void ) ;

/* sketch */
void setup( void ) ;
void loop( void ) ;

#include "WVariant.h"

#ifdef __cplusplus
} // extern "C"
#endif

// The following headers are for C++ only compilation
#ifdef __cplusplus
  #include "WCharacter.h"
  #include "WString.h"
  // #include "Tone.h"
  #include "WMath.h"
  #include "HardwareSerial.h"
  #include "pulse.h"
#endif
#include "delay.h"
#include "binary.h"
#ifdef __cplusplus
  #include "Uart.h"
#endif

#include "elapsedMillis.h"


// Include board variant
#include "variant.h"

#include "wiring.h"
#include "wiring_digital.h"
#include "wiring_analog.h"
#include "wiring_shift.h"
#include "WInterrupts.h"

// undefine stdlib's abs if encountered
#ifdef abs
#undef abs
#endif // abs

#ifdef __cplusplus
#include <type_traits>
#include <utility>
// when the input number is an integer type, do all math as 32 bit signed long
template <class T, class A, class B, class C, class D>
long map(T _x, A _in_min, B _in_max, C _out_min, D _out_max, typename std::enable_if<std::is_integral<T>::value >::type* = 0)
{
	long x = _x, in_min = _in_min, in_max = _in_max, out_min = _out_min, out_max = _out_max;
	if ((in_max - in_min) > (out_max - out_min)) {
		return (x - in_min) * (out_max - out_min+1) / (in_max - in_min+1) + out_min;
	} else {
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}
}
// when the input is a float or double, do all math using the input's type
template <class T, class A, class B, class C, class D>
T map(T x, A in_min, B in_max, C out_min, D out_max, typename std::enable_if<std::is_floating_point<T>::value >::type* = 0)
{
	return (x - (T)in_min) * ((T)out_max - (T)out_min) / ((T)in_max - (T)in_min) + (T)out_min;
}

template<class A, class B>
constexpr auto min(A&& a, B&& b) -> decltype(a < b ? std::forward<A>(a) : std::forward<B>(b)) {
    return a < b ? std::forward<A>(a) : std::forward<B>(b);
}

template<class A, class B>
constexpr auto max(A&& a, B&& b) -> decltype(a < b ? std::forward<A>(a) : std::forward<B>(b)) {
    return a > b ? std::forward<A>(a) : std::forward<B>(b);
}
#else
  //#define min(a,b) ((a)<(b)?(a):(b))
  //#define max(a,b) ((a)>(b)?(a):(b))
  #define min(a, b) ({ \
  typeof(a) _a = (a); \
  typeof(b) _b = (b); \
  (_a < _b) ? _a : _b; \
  })
  #define max(a, b) ({ \
  typeof(a) _a = (a); \
  typeof(b) _b = (b); \
  (_a > _b) ? _a : _b; \
  })
#endif

#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))

#define interrupts() __enable_irq()
#define noInterrupts() __disable_irq()

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define bit(b) (1UL << (b))

//#define digitalPinToPort(P)        ( &(NRF_GPIO[P]) ) //removed for software serial
#define digitalPinToPort(P)        ( NRF_GPIO ) // NRF_P0 = P0.00 - P0.31, NRF_P1 = P1.00 - P1.15 (e.g. nRF52840)
#define digitalPinToBitMask(P)     ( 1 << g_ADigitalPinMap[P] )
//#define analogInPinToBit(P)        ( )
#define portOutputRegister(port)   ( &(port->OUT) )
#define portInputRegister(port)    ( &(port->IN) )
#define portModeRegister(port)     ( &(port->DIR) )
#define digitalPinHasPWM(P)        ( true )

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for nRF52
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https://github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)

// Interrupts
#define digitalPinToInterrupt(P)   ( P )

#ifdef __cplusplus
#include "Uart.h"
#endif // __cplusplus

#endif // Arduino_h
