/******      Web server config      ******/
#include "webIO.h"

bool shouldReboot = false;
AsyncWebServer webServer(PORT_WEBSERVER);
AsyncWebSocket wsSensorData("/data"), wsConsole("/console");


/***************************************************/
/******            SETUP FUNCTIONS            ******/
/***************************************************/
void setupFileSystem() {	// Initialize file system, so we can read/write config and web files
	if (!SPIFFS.begin(true)) {
		consolePrintF("Failed to mount file system!!\n");
	} else {
		consolePrintF("SPIFFS loaded!\n");
	}
}

void setupMDNS() {	// Setup mDNS so we don't need to know its IP
	#if USE_MDNS
		consolePrintF("Starting mDNS... ");
		if (MDNS.begin(hostName)) {
			consolePrintF("Started! you can now also contact me through 'http://%s.local'\n", hostName);
			MDNS.addService(F("http"), F("tcp"), PORT_WEBSERVER);
		} else {
			consolePrintF("Unable to start mDNS :(\n");
		}
	#endif
}

void setupWebServer() {	// Initialize webServer and setup handlers for each url, as well as webSockets
	// Configure urls that are handled "manually" (not SPIFFS files)
	webServer.on(CF("/WiFiNets"), HTTP_GET, webServerWLANscan);
	webServer.on(CF("/WiFiSave"), HTTP_POST, webServerWLANsave);
	webServer.on(CF("/heap"), HTTP_GET, [](AsyncWebServerRequest* request) { AsyncWebServerResponse* response = request->beginResponse(200, SF("text/plain"), String(ESP.getFreeHeap()) + F(" B")); addNoCacheHeaders(response); response->addHeader(F("Refresh"), F("2")); request->send(response); });
	webServer.on(CF("/restart"), HTTP_GET, [](AsyncWebServerRequest* request) { AsyncWebServerResponse* response = request->beginResponse(200, SF("text/plain"), F("Restarting! You'll be automatically redirected to /heap in 15s to make sure it rebooted ;)")); addNoCacheHeaders(response); response->addHeader(F("Refresh"), F("15; url=/heap")); request->send(response); shouldReboot = true; });

	// Serve files from "/www" folder in SPIFFS
	webServer.serveStatic("/", SPIFFS, "/www/", "public, max-age=1209600").setDefaultFile("index.html");	// Cache for 2 weeks :)
	webServer.rewrite("/WiFi", "/WiFi.html");
	webServer.rewrite("/protos/google/protobuf/descriptor.proto", "/protos/descriptor.proto");	// Workaround for SPIFFS's

	// Allow user to upload their own files to SPIFFS through the server (urls: /upload or /edit)
	webServer.on(CF("/upload"), HTTP_GET, [](AsyncWebServerRequest* request){ if (SPIFFS.exists(F("/www/upload.html"))) { request->send(SPIFFS, F("/www/upload.html")); } else { request->send(200, SF("text/html"), F("<html><head><link rel=\"stylesheet\" href=\"css/styles.css\"></head><body><h1>Secret file uploader</h1><form method=\"POST\" action=\"upload\" enctype=\"multipart/form-data\"><p>New file name: <input type=\"text\" placeholder=\"/\" name=\"fileName\" value=\"\" /><br><input type=\"file\" name=\"fileContent\" value=\"\" /><br><input type=\"submit\" value=\"Upload file\" /></p></form></body></html>")); } });
	webServer.on(CF("/upload"), HTTP_POST, [](AsyncWebServerRequest* request){ bool ok = renameFileUpload(request->getParam(F("fileName"), true)->value()); request->redirect(SF("upload?f=") + request->getParam(F("fileName"), true)->value() + F("&ok=") + ok); }, handleFileUpload);
	webServer.addHandler(new SPIFFSEditor(SPIFFS, WEB_FILE_EDITOR_USERNAME, WEB_FILE_EDITOR_PASS));

	webServer.onNotFound([](AsyncWebServerRequest* request) { request->send(404, SF("text/plain"), SF("Not found: ") + request->url()); });

	// Setup web sockets
	wsSensorData.onEvent(onWsEvent);
	webServer.addHandler(&wsSensorData);
	wsConsole.onEvent(onWsEvent);
	webServer.addHandler(&wsConsole);

	// And start the server
	webServer.begin();
}

void setupOTA() {	// Setup Arduino's OTA (to upload new firmware through their IDE)
#if USE_ARDUINO_OTA
	ArduinoOTA.setHostname(hostName);
	ArduinoOTA.setPort(PORT_ARDUINO_OTA);
	ArduinoOTA.setPassword(WEB_FILE_EDITOR_PASS.c_str());
	ArduinoOTA.onStart([]() {
		consolePrintF("OTA: Start!\n");
	});
	ArduinoOTA.onEnd([]() {
		consolePrintF("OTA: Firmware update succeeded!\n");
	});
	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
		consolePrintF("OTA: %3u%% completed (%4u KB / %4u KB)\r", progress / (total/100), progress>>10, total>>10);
	});
	ArduinoOTA.onError([](ota_error_t error) {
		consolePrintF("\nOTA: Error[%u]: ", error);
		if (error == OTA_AUTH_ERROR) consolePrintF("Authentication failed\n");
		else if (error == OTA_BEGIN_ERROR) consolePrintF("Begin failed\n");
		else if (error == OTA_CONNECT_ERROR) consolePrintF("Connection failed\n");
		else if (error == OTA_RECEIVE_ERROR) consolePrintF("Reception failed\n");
		else if (error == OTA_END_ERROR) consolePrintF("End failed\n");

		consolePrintF("Rebooting Arduino...\n");
		ESP.restart();
	});
	ArduinoOTA.begin();
#endif
}

void setupWebIO() {	// Initializes mDNS, HTTP server, webSockets and OTA
	setupFileSystem();
#if (!USE_SERIAL_INSTEAD_OF_WIFI)
	setupMDNS();
	setupWebServer();
	setupOTA();
#endif
}


/*********************************************************/
/******      	Web server related functions      	******/
/*********************************************************/
void addNoCacheHeaders(AsyncWebServerResponse* response) {	// Add specific headers to an http response to avoid caching
	response->addHeader(F("Cache-Control"), F("no-cache, no-store, must-revalidate"));
	response->addHeader(F("Pragma"), F("no-cache"));
	response->addHeader(F("Expires"), F("-1"));
}

void handleFileUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {	// Allows user to upload a file to the SPIFFS (so we don't have to write the whole Flash via USB)
	static File fUpload;

	if (!index) {
		consolePrintF("\nUploading new SPIFFS file (to the temporary path %s) with filename %s\n", UPLOAD_TEMP_FILENAME, filename.c_str());
		fUpload = SPIFFS.open(UPLOAD_TEMP_FILENAME, "w");
	}

	if (fUpload)
		fUpload.write(data, len);
	if (final) {
		if (fUpload)
			fUpload.close();
		consolePrintF("Successfully uploaded new SPIFFS file with filename %s (%u B)!\n", filename.c_str(), index+len);
	}
}

bool renameFileUpload(String fileName) {	// Renames the last file uploaded to the new file name provided
	if (!fileName.startsWith("/"))
		fileName = "/" + fileName;
	if (SPIFFS.exists(fileName)) {
		SPIFFS.remove(fileName);
		consolePrintF("\t(File already existed, removing old version -> Overwriting)\n");
	}

	bool r = SPIFFS.rename(UPLOAD_TEMP_FILENAME, fileName);
	consolePrintF("%s temporary file %s -> %s\n\n", (r? CF("Successfully renamed"):CF("Couldn't rename")), UPLOAD_TEMP_FILENAME, fileName.c_str());

	return r;
}

void webServerWLANscan(AsyncWebServerRequest* request) {	// Handles secret HTTP page that scans WLAN networks
	String json = SF("{\"currAP\":{\"ssid\":\"") + hostName + F("\",\"ip\":\"") + WiFi.softAPIP().toString() + F("\"},"
		"\"currWLAN\":{\"ssid\":\"") + String(wlanSSID) + F("\",\"pass\":\"") + String(wlanPass) + F("\",\"staticIp\":\"") + String(wlanStaticIP) + F("\",\"ip\":\"") + wlanMyIP.toString() + F("\",\"gateway\":\"") + wlanGateway.toString() + F("\",\"mask\":\"") + wlanMask.toString() + F("\"},"
		"\"nets\":[");
	int n = WiFi.scanComplete();

	consolePrintF("Scanning WiFi... n=%d\n", n);
	if (n == -2) {	// Scan not triggered
		WiFi.scanNetworks(true);	// Trigger a scan
	} else if (n>=0) {
		for (int i=0; i<n; ++i) {
			if (i) json += ",";
			json += SF("{"
				"\"rssi\":") + String(WiFi.RSSI(i)) + F(","
				"\"ssid\":\"") + WiFi.SSID(i) + F("\","
				"\"bssid\":\"") + WiFi.BSSIDstr(i) + F("\","
				"\"channel\":") + String(WiFi.channel(i)) + F(","
				"\"secure\":") + String(WiFi.encryptionType(i)) + F(
			"}");
		}
		WiFi.scanDelete();
		if (WiFi.scanComplete() == -2) {	// Restart scan
			WiFi.scanNetworks(true);
		}
	}

	json += "]}";
	AsyncWebServerResponse* response = request->beginResponse(200, SF("text/json"), json);
	addNoCacheHeaders(response);
	request->send(response);
	json = String();
}

void webServerWLANsave(AsyncWebServerRequest* request) {	// Handles secret HTTP page that saves new WLAN settings
	consolePrintF("Received request to save new WLAN settings!\n");
	bool bSSIDmanual = false;
	if (request->hasParam(F("ssidManualChk"), true)) bSSIDmanual = (request->getParam(F("ssidManualChk"), true)->value()=="on");
	if (request->hasParam(bSSIDmanual? CF("ssidManualTxt"):CF("ssidDropdown"), true)) request->getParam(bSSIDmanual? F("ssidManualTxt"):F("ssidDropdown"), true)->value().toCharArray(wlanSSID, sizeof(wlanSSID)-1);
	if (request->hasParam(F("pass"), true)) request->getParam(F("pass"), true)->value().toCharArray(wlanPass, sizeof(wlanPass)-1);
	wlanStaticIP = (request->hasParam(F("staticIp"), true));
	if (request->hasParam(F("ip"), true)) wlanMyIP.fromString(request->getParam(F("ip"), true)->value());
	if (request->hasParam(F("gateway"), true)) wlanGateway.fromString(request->getParam(F("gateway"), true)->value());
	if (request->hasParam(F("mask"), true)) wlanMask.fromString(request->getParam(F("mask"), true)->value());
	saveWLANconfig();	// Write new settings to EEPROM

	String strIPinfo = (!wlanStaticIP? SF("Dynamic IP") : (SF("IP: ") + wlanMyIP.toString() + F("<br>Gateway: ") + wlanGateway.toString() + F("<br>Mask: ") + wlanMask.toString()));
	AsyncWebServerResponse* response = request->beginResponse(200, SF("text/html"), SF("<html><head><link rel=\"stylesheet\" href=\"css/styles.css\"></head><body><h1>WiFi config successfully saved!</h1><p>SSID: ") + wlanSSID + F("<br>") + strIPinfo + F("</p></body></html>"));
	addNoCacheHeaders(response);
	request->send(response);
}

void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t * data, size_t len) {	// webSocket event callback function
	String msg;
	AwsFrameInfo *info = (AwsFrameInfo*)arg;

	switch(type) {
	case WS_EVT_CONNECT:
		consolePrintF("[WebSocket '%s'] Client connected from %s:%d (clientId %u)\n", server->url(), client->remoteIP().toString().c_str(), client->remotePort(), client->id());
		client->text(hostName);	// Send message to client to confirm connection ok
		break;
	case WS_EVT_DISCONNECT:
		consolePrintF("[WebSocket '%s'] ClientId %u (%s:%d) disconnected!\n", server->url(), client->id(), client->remoteIP().toString().c_str(), client->remotePort());
		break;
	case WS_EVT_ERROR:
    	consolePrintF("[WebSocket '%s'] ClientId %u error(%u): %s\n", server->url(), client->id(), *((uint16_t*)arg), (char*)data);
    	break;
	case WS_EVT_DATA:
		if(info->opcode == WS_TEXT) {
			msg = String((char *) data);
		} else {	// Decode binary message
			char buff[3];
			for (size_t i=0; i<info->len; i++) {
				sprintf(buff, "%02x ", (uint8_t) data[i]);
				msg += buff;
			}
		}

		consolePrintF("[WebSocket '%s'] Message received from clientId %u (%s:%d):\n\t%s\n", server->url(), client->id(), client->remoteIP().toString().c_str(), client->remotePort(), msg.c_str());
		break;
    default:
    	break;
	}
}

void consolePrintf(const char * format, ...) {	// Log messages through webSocketConsole and Serial
	char buf[1024];

	// Emulate printf() with variable arguments and save output on buf
	va_list args;
    va_start(args, format);
    size_t len = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

	// Output buf through both the WebSocket and Serial port
	wsConsole.textAll(buf, len);
	Serial.printf(buf);
}

void processWebIO() {	// "webIO.loop()" function: handle incoming OTA connections (if any), http requests and webSocket events
	uint32_t t_msec = curr_time%1000, t_sec = curr_time/1000, t_min = t_sec/60, t_hr = t_min/60; t_sec %= 60; t_min %= 60;
	static uint32_t last_t_sec = 0;
	if (t_sec != last_t_sec) {	// Every second, log that we are alive
		last_t_sec = t_sec;
		#if (!USE_SERIAL_INSTEAD_OF_WIFI)	// If we're collecting data through serial, don't print time
			consolePrintF("Still alive (t=%3d:%02d'%02d\"); HEAP: %5d B\n", t_hr, t_min, t_sec, ESP.getFreeHeap());
		#endif
	}

	if (shouldReboot) ESP.restart();	// AsyncWebServer doesn't suggest rebooting from async callbacks, so we set a flag and reboot from here :)

	#if USE_ARDUINO_OTA
		ArduinoOTA.handle();
	#endif
}
