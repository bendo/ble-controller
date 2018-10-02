#include <SPI.h>
#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <bluefruit.h>
#include "Adafruit_seesaw.h"


#define VBAT_PIN          (A7)
#define VBAT_MV_PER_LSB   (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#define VBAT_DIVIDER      (0.71275837F)   // 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
#define VBAT_DIVIDER_COMP (1.403F)        // Compensation factor for the VBAT divider

Adafruit_SSD1306 display = Adafruit_SSD1306();
#if defined(ESP8266)
  #define BUTTON_A 0
  #define BUTTON_B 16
  #define BUTTON_C 2
  #define LED      0
#elif defined(ESP32)
  #define BUTTON_A 15
  #define BUTTON_B 32
  #define BUTTON_C 14
  #define LED      13
#elif defined(ARDUINO_STM32F2_FEATHER)
  #define BUTTON_A PA15
  #define BUTTON_B PC7
  #define BUTTON_C PC5
  #define LED PB5
#elif defined(TEENSYDUINO)
  #define BUTTON_A 4
  #define BUTTON_B 3
  #define BUTTON_C 8
  #define LED 13
#elif defined(ARDUINO_FEATHER52)
  #define BUTTON_A 31
  #define BUTTON_B 30
  #define BUTTON_C 27
  #define LED 17
#else // 32u4, M0, M4, and 328p
  #define BUTTON_A 9
  #define BUTTON_B 6
  #define BUTTON_C 5
  #define LED      13
#endif

Adafruit_seesaw ss;
#define BUTTON_RIGHT 6
#define BUTTON_DOWN  7
#define BUTTON_LEFT  9
#define BUTTON_UP    10
#define BUTTON_START   14
uint32_t button_mask = (1 << BUTTON_RIGHT) | (1 << BUTTON_DOWN) |
(1 << BUTTON_LEFT) | (1 << BUTTON_UP) | (1 << BUTTON_START);
#if defined(ESP8266)
#define IRQ_PIN   2
#elif defined(ESP32)
#define IRQ_PIN   14
#elif defined(NRF52)
#define IRQ_PIN   27
#elif defined(TEENSYDUINO)
#define IRQ_PIN   8
#elif defined(ARDUINO_ARCH_WICED)
#define IRQ_PIN   PC5
#else
#define IRQ_PIN   5
#endif

BLEClientDis clientDis;
BLEClientUart clientUart;

void setup(void)
{
	Serial.begin(115200);

	setupOLED();
	setupJoy();
	setupBLE();
}

int last_x = 0, last_y = 0;

char direction[32] = {0};

char peer_name[32] = {0};

void loop()
{
	while(true) {
		printBatteryLife();
		controlOLED();
		if (Bluefruit.Central.connected()) {
			if (clientUart.discovered()) {

				int y = ss.analogRead(2);
				int x = ss.analogRead(3);

				if(x > 600 && last_x < 600){
					sendMessage("81");
					strcpy(direction, "RIGHT");
				}
				else if(last_x > 600 && x < 600){
					sendMessage("80");
					strcpy(direction, "");
				}
				if(x < 400 && last_x > 400){
					sendMessage("71");
					strcpy(direction, "LEFT");
				}
				else if(last_x < 400 && x > 400){
					sendMessage("70");
					strcpy(direction, "");
				}
				if(y > 600 && last_y < 600){
					sendMessage("61");
					strcpy(direction, "REAR");
				}
				else if(last_y > 600 && y < 600){
					sendMessage("60");
					strcpy(direction, "");
				}
				if(y < 400 && last_y > 400){
					sendMessage("51");
					strcpy(direction, "FRONT");
				}
				else if(last_y < 400 && y > 400){
					sendMessage("50");
					strcpy(direction, "");
				}

				if ( (abs(x - last_x) > 3)  ||  (abs(y - last_y) > 3)) {
					last_x = x;
					last_y = y;
				}
				/*
				   if(!digitalRead(IRQ_PIN)){
				   uint32_t buttons = ss.digitalReadBulk(button_mask);
				//Serial.println(buttons, BIN);
				if (! (buttons & (1 << BUTTON_DOWN))) {
				sendMessage("B11");
				}
				else {
				sendMessage("B00");
				}
				if (! (buttons & (1 << BUTTON_RIGHT))) {
				sendMessage("A11");
				}
				else {
				sendMessage("A00");
				}

				if (! (buttons & (1 << BUTTON_LEFT))) {
				sendMessage("Y11");
				}
				else {
				sendMessage("Y00");
				}
				if (! (buttons & (1 << BUTTON_UP))) {
				sendMessage("X11");
				}
				else {
				sendMessage("X00");
				}
				if (! (buttons & (1 << BUTTON_START))) {
				sendMessage("S11");
				}
				else {
				sendMessage("S00");
				}
				}
				*/			}
		}
		else {
			strcpy(peer_name, "");
		}
	}
}

void sendMessage(const char* msg)
{
	int i = 0;
	while(i<10) {
		clientUart.print(msg);
		Serial.println(msg);
		delay(4);
		i++;
	}
}

void setupBLE(void)
{
	Bluefruit.begin(0, 1);
	Bluefruit.setName("bendo's controller");

	clientDis.begin();

	clientUart.begin();
	clientUart.setRxCallback(bleuart_rx_callback);

	Bluefruit.setConnLedInterval(250);

	Bluefruit.Central.setConnectCallback(connect_callback);
	Bluefruit.Central.setDisconnectCallback(disconnect_callback);

	Bluefruit.Scanner.setRxCallback(scan_callback);
	Bluefruit.Scanner.restartOnDisconnect(true);
	Bluefruit.Scanner.setInterval(160, 80);
	Bluefruit.Scanner.useActiveScan(false);
	Bluefruit.Scanner.start(0);
}

void scan_callback(ble_gap_evt_adv_report_t* report)
{
	if (Bluefruit.Scanner.checkReportForService(report, clientUart)) {
		Serial.print("BLE UART service detected. Connecting ... ");
		Bluefruit.Central.connect(report);
	}
}

void connect_callback(uint16_t conn_handle)
{
	Serial.println("Connected");

	Serial.print("Dicovering DIS...");
	if (clientDis.discover(conn_handle)) {
		char buffer[32+1];

		memset(buffer, 0, sizeof(buffer));
		if (clientDis.getManufacturer(buffer, sizeof(buffer))) {
			Serial.print("Manufacturer: ");
			Serial.println(buffer);
		}

		memset(buffer, 0, sizeof(buffer));
		if (clientDis.getModel(buffer, sizeof(buffer)))	{
			Serial.print("Model: ");
			Serial.println(buffer);
		}

		Serial.println();
	}

	Bluefruit.Gap.getPeerName(conn_handle, peer_name, sizeof(peer_name));

	Serial.print("Connected to ");
	Serial.println(peer_name);

	if (clientUart.discover(conn_handle)) {
		clientUart.enableTXD();
		loop();
	} else {
		Bluefruit.Central.disconnect(conn_handle);
	}
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
	(void) conn_handle;
	(void) reason;


	Serial.println("Disconnected");
}

void bleuart_rx_callback(BLEClientUart& uart_svc)
{
	// dead code???
	Serial.println();
}

void setupJoy(void)
{
	if (!ss.begin(0x49)){
		Serial.println("ERROR!");
		while(1);
	}
	else {
		Serial.println("seesaw started");
		Serial.print("version: ");
		Serial.println(ss.getVersion(), HEX);
	}
	ss.pinModeBulk(button_mask, INPUT_PULLUP);
	ss.setGPIOInterrupts(button_mask, 1);
	pinMode(IRQ_PIN, INPUT);

}

void setupOLED(void)
{
	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

	pinMode(BUTTON_A, INPUT_PULLUP);
	pinMode(BUTTON_B, INPUT_PULLUP);
	pinMode(BUTTON_C, INPUT_PULLUP);
}

void controlOLED(void)
{
	if (!digitalRead(BUTTON_A)) {
		// doesn't work
	}
	if (!digitalRead(BUTTON_B)) {
		// doesn't work
		//	printBatteryLife();
	}
	if (!digitalRead(BUTTON_C)) {
		// doesn't work
		//	display.clearDisplay();
	}
}

void printBatteryLife()
{
	int vbat_raw = readVBAT();
	float vbat_mv = (float)vbat_raw * VBAT_MV_PER_LSB * VBAT_DIVIDER_COMP;
	uint8_t vbat_per = mvToPercent(vbat_raw * VBAT_MV_PER_LSB);

	display.clearDisplay();
	display.setTextSize(1);
	display.setTextColor(WHITE);
	display.setCursor(0, 0);
	delay(4);
	display.print(peer_name);
	display.setCursor(0, 11);
	delay(4);
	display.print("LIPO: ");
	display.print(vbat_mv);
	display.print("mV - ");
	display.print(vbat_per);
	display.print("%");
	display.setCursor(0, 22);
	delay(4);
	display.print(direction);
	display.display();
	delay(4);
}

int readVBAT(void) {
	int raw;
	analogReference(AR_INTERNAL_3_0);
	analogReadResolution(12); // Can be 8, 10, 12 or 14
	delay(1);
	raw = analogRead(VBAT_PIN);
	analogReference(AR_DEFAULT);
	analogReadResolution(10);
	return raw;
}

uint8_t mvToPercent(float mvolts) {
	uint8_t battery_level;

	if (mvolts >= 3000) {
		battery_level = 100;
	}
	else if (mvolts > 2900) {
		battery_level = 100 - ((3000 - mvolts) * 58) / 100;
	}
	else if (mvolts > 2740) {
		battery_level = 42 - ((2900 - mvolts) * 24) / 160;
	}
	else if (mvolts > 2440) {
		battery_level = 18 - ((2740 - mvolts) * 12) / 300;
	}
	else if (mvolts > 2100) {
		battery_level = 6 - ((2440 - mvolts) * 6) / 340;
	}
	else {
		battery_level = 0;
	}
	return battery_level;
}
