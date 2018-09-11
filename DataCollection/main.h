/***********       IoT pairing project - Main include       ***********/
/* License:  MIT                                                      */
/* Author:   Carlos Ruiz (carlos.r.domin@gmail.com, carlosrd@cmu.edu) */
#ifndef MAIN_H_
#define MAIN_H_

#include <Arduino.h>	// Aruino general includes and definitions
#include "webIO.h"		// In case we want to print debug messages through a webSocket (consolePrintF method)

#define USE_SERIAL_INSTEAD_OF_WIFI	true			// If true, sensor data is sent over Serial port instead of over WiFi
#define SF(literal)		String(F(literal))			// Macro to save a string literal in Flash memory and convert it to String when reading it
#define CF(literal)		String(F(literal)).c_str()	// Macro to save a string literal in Flash memory and convert it to char* when reading it
#define SFPSTR(progmem)	String(FPSTR(progmem))		// Macro to convert a PROGMEM string (in Flash memory) to String

extern uint32_t curr_time;	// Global variable to track time at the beginning of each loop() iteration
extern char hostName[32];	// Unique hostname derived from the serial number (used as SOFT_AP_SSID, for mDNS, etc.). Defined and initialized in WiFi.cpp

#endif
