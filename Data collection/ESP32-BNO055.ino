#include "main.h"
#include "WiFi.h"
#include "webIO.h"
#include "GPIO.h"

uint32_t curr_time;


/***************************************************/
/******            SETUP FUNCTIONS            ******/
/***************************************************/
void setup(void) {
	curr_time = millis();
    Serial.begin(115200);
    Serial.setDebugOutput(false);	// Print internal library debug messages through Serial (for debugging)
	while(!Serial);	// Wait for serial port to connect

    setupWiFi();
    setupWebIO();
    BNO055::setup();
	consolePrintF("\n -- SETUP COMPLETE -- \n");	// Indicate that setup is done (we'll only send sensor data over Serial from now on #if USE_SERIAL_INSTEAD_OF_WIFI)
}


/*********************************************/
/******            MAIN LOOP            ******/
/*********************************************/
void loop(void) {
	curr_time = millis();
	processWiFi();
    processWebIO();
	delay(20);
}
