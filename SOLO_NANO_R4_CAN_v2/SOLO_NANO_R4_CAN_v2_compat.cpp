/*
  SOLOMotorControllers 5.5.0 enables its native CANopen implementation for
  UNO R4 Minima/WiFi but omits ARDUINO_NANO_R4 from the same compatibility
  guards. Nano R4 uses the same RA4M1 HardwareCAN API, so compile those two
  implementation units locally until the library adds Nano R4 support.
*/

#if defined(ARDUINO_NANO_R4)
#define ARDUINO_MINIMA
#include <CanBus.cpp>
#include <SOLOMotorControllersCanopenNative.cpp>
#endif
