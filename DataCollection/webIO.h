/******      Web server config      ******/
#ifndef WEB_IO_H_
#define WEB_IO_H_

#include "main.h"					// Global includes and definitions
#include "WiFi.h"					// WiFi library needed to access WLAN config settings (wlanSSID, wlanPass, etc.)
#include <ESPAsyncWebServer.h>		// Async HTTP web server (includes websocket and SSE support)
#include <SPIFFS.h>					// SPIFFS file system (to read/write to flash)
#include <SPIFFSEditor.h>			// Helper that provides the resources to view&edit SPIFFS files through HTTP

#define consolePrintF(s, ...)		consolePrintf(CF(s), ##__VA_ARGS__)
#define PORT_WEBSERVER				80		// Port for the webServer
#define PORT_ARDUINO_OTA			3232	// Port recognized by Arduino IDE which allows remote firmware flashing
#define WEB_FILE_EDITOR_USERNAME	SF("PEILab")
#define WEB_FILE_EDITOR_PASS		SF("IoTpairing")
#define USE_ARDUINO_OTA				true	// Whether or not to use Arduino's native IDE remote firmware flasher
#define USE_MDNS					true	// Whether or not to use mDNS (allows access to the arduino through a name without knowing its IP)
#define UPLOAD_TEMP_FILENAME		"/tmp.file"	// Temporary file name given to a file uploaded through the web server. Once we receive its desired path, we'll rename it (move it)

#if USE_ARDUINO_OTA
#include <ArduinoOTA.h>				// OTA (upload firmware through Arduino IDE over WiFi)
#endif

#if USE_MDNS
#include <ESPmDNS.h>				// DNS (allows access to the arduino through a name without knowing its IP)
#endif

extern AsyncWebSocket wsSensorData;


/***************************************************/
/******            SETUP FUNCTIONS            ******/
/***************************************************/
void setupWebIO();	// Initializes hostName, mDNS, HTTP server, OTA and webSockets


/*********************************************************/
/******      	Web server related functions      	******/
/*********************************************************/
void addNoCacheHeaders(AsyncWebServerResponse* response);	// Add specific headers to an http response to avoid caching
void handleFileUpload(AsyncWebServerRequest* request, String filename, size_t index, uint8_t * data, size_t len, bool final);	// Allows user to upload a file to the SPIFFS (so we don't have to write the whole Flash via USB)
bool renameFileUpload(String fileName);	// Renames the last file uploaded to the new file name provided
void webServerWLANscan(AsyncWebServerRequest* request);	// Handles secret HTTP page that scans WLAN networks
void webServerWLANsave(AsyncWebServerRequest* request);	// Handles secret HTTP page that saves new WLAN settings
void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t * data, size_t len);	// webSocket event callback function
void consolePrintf(const char * format, ...);	// Log messages through webSocketConsole and Serial
void processWebIO();	// "webServer.loop()" function: handle incoming OTA connections (if any), http requests and webSocket events

#endif
