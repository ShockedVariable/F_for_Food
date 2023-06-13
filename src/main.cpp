#include <ArduinoJson.h>
#include <base64.h>
#include <EEPROM.h>
#include <ESPAsyncWebServer.h>
#include <MQTTClient.h>
#include <WiFiClientSecure.h>
#include "Arduino.h"
#include "driver/rtc_io.h"
#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "secrets.hpp"
#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include "WiFi.h"

// Button Stuff
#define BUTTON 12
int buttonState = HIGH;
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

boolean takeNewPhoto = false;

// Photo File Name to save in SPIFFS
#define FILE_PHOTO "/photo.jpg"

#define AWS_IOT_PUBLISH_TOPIC   "detectfood/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "detectfood/sub"

// OV2640 camera module pins (CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

WiFiClientSecure net = WiFiClientSecure();
MQTTClient client = MQTTClient(15000);

// Check if photo capture was successful
bool checkPhoto( fs::FS &fs ) 
{
    File f_pic = fs.open( FILE_PHOTO );
    unsigned int pic_sz = f_pic.size();
    return ( pic_sz > 100 );
}

void lwMQTTErr(lwmqtt_err_t reason)
{
	if (reason == lwmqtt_err_t::LWMQTT_SUCCESS)
		Serial.print("Success");
	else if (reason == lwmqtt_err_t::LWMQTT_BUFFER_TOO_SHORT)
		Serial.print("Buffer too short");
	else if (reason == lwmqtt_err_t::LWMQTT_VARNUM_OVERFLOW)
		Serial.print("Varnum overflow");
	else if (reason == lwmqtt_err_t::LWMQTT_NETWORK_FAILED_CONNECT)
		Serial.print("Network failed connect");
	else if (reason == lwmqtt_err_t::LWMQTT_NETWORK_TIMEOUT)
		Serial.print("Network timeout");
	else if (reason == lwmqtt_err_t::LWMQTT_NETWORK_FAILED_READ)
		Serial.print("Network failed read");
	else if (reason == lwmqtt_err_t::LWMQTT_NETWORK_FAILED_WRITE)
		Serial.print("Network failed write");
	else if (reason == lwmqtt_err_t::LWMQTT_REMAINING_LENGTH_OVERFLOW)
		Serial.print("Remaining length overflow");
	else if (reason == lwmqtt_err_t::LWMQTT_REMAINING_LENGTH_MISMATCH)
		Serial.print("Remaining length mismatch");
	else if (reason == lwmqtt_err_t::LWMQTT_MISSING_OR_WRONG_PACKET)
		Serial.print("Missing or wrong packet");
	else if (reason == lwmqtt_err_t::LWMQTT_CONNECTION_DENIED)
		Serial.print("Connection denied");
	else if (reason == lwmqtt_err_t::LWMQTT_FAILED_SUBSCRIPTION)
		Serial.print("Failed subscription");
	else if (reason == lwmqtt_err_t::LWMQTT_SUBACK_ARRAY_OVERFLOW)
		Serial.print("Suback array overflow");
	else if (reason == lwmqtt_err_t::LWMQTT_PONG_TIMEOUT)
		Serial.print("Pong timeout");
}

void lwMQTTErrConnection(lwmqtt_return_code_t reason)
{
	if (reason == lwmqtt_return_code_t::LWMQTT_CONNECTION_ACCEPTED)
		Serial.print("Connection Accepted");
	else if (reason == lwmqtt_return_code_t::LWMQTT_UNACCEPTABLE_PROTOCOL)
		Serial.print("Unacceptable Protocol");
	else if (reason == lwmqtt_return_code_t::LWMQTT_IDENTIFIER_REJECTED)
		Serial.print("Identifier Rejected");
	else if (reason == lwmqtt_return_code_t::LWMQTT_SERVER_UNAVAILABLE)
		Serial.print("Server Unavailable");
	else if (reason == lwmqtt_return_code_t::LWMQTT_BAD_USERNAME_OR_PASSWORD)
		Serial.print("Bad UserName/Password");
	else if (reason == lwmqtt_return_code_t::LWMQTT_NOT_AUTHORIZED)
		Serial.print("Not Authorized");
	else if (reason == lwmqtt_return_code_t::LWMQTT_UNKNOWN_RETURN_CODE)
		Serial.print("Unknown Return Code");
}

void takePictureAndSubmit()
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) 
    {
        Serial.println("Camera capture failed");
        return;
    } 
    else 
    {
        Serial.println("Camera capture successful!");
    }

	const char* data = (const char*) fb->buf;
	// Image metadata.  Yes it should be cleaned up to use printf if the function is available
	Serial.print("Size of image:");
	Serial.println(fb->len);
	Serial.print("Shape->width:");
	Serial.print(fb->width);
	Serial.print("height:");
	Serial.println(fb->height);

	String encoded = base64::encode(fb->buf, fb->len);

	// formating json
	DynamicJsonDocument doc(10000);
	doc["picture"] = encoded;
	String jsonBuffer;

	serializeJson(doc, jsonBuffer);

	Serial.println(jsonBuffer);

	// publishing topic
	if (!client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer))
	{
		lwMQTTErr(client.lastError());
	}
	Serial.println("Published");

	// Killing cam resource
	esp_camera_fb_return(fb);
}

void pushButton()
{
    int button_reading = digitalRead(BUTTON);

    if (button_reading != lastButtonState)
    {
        lastDebounceTime = millis();
    }

    // Check if the debounce delay has passed
    if ((millis() - lastDebounceTime) > debounceDelay) 
    {
        // Update the button state if it has stabilized
        if (button_reading != buttonState) 
        {
            buttonState = button_reading;

            // Perform actions based on the button state change
            if (buttonState == LOW) 
            {
                // Button is pressed
                // Perform desired actions
				takePictureAndSubmit();
            } 
        }
    }

    // Update the last button state
    lastButtonState = button_reading;
}

void messageHandler(String& topic, String& payload)
{
    Serial.println("incoming: " + topic + " - " + payload);
}

void connectAWS()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) 
	{
		delay(500);
		Serial.println("Connecting to WiFi...");
	}

    // Configure WiFiClientSecure to use the AWS IoT device credentials
    net.setCACert(AWS_CERT_CA);
    net.setCertificate(AWS_CERT_CRT);
    net.setPrivateKey(AWS_CERT_PRIVATE);

    // Connect to the MQTT broker on the AWS endpoint we defined earlier
    client.begin(AWS_IOT_ENDPOINT, 8883, net);

    // Create a message handler
    client.onMessage(messageHandler);

    Serial.print("Connecting to AWS IOT");

    while (!client.connect(THINGNAME)) 
    {
        Serial.print(".");
        delay(100);
    }

    if(!client.connected())
    {
        Serial.println("AWS IoT Timeout!");
        return;
    }

    // Subscribe to a topic
    client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);

    Serial.println("AWS IoT Connected!");
}

void setup() 
{
	// Serial port for debugging purposes
	Serial.begin(115200);

	pinMode(BUTTON, INPUT_PULLUP);
	// initCam();
	// OV2640 camera module
	camera_config_t config;
	config.ledc_channel = LEDC_CHANNEL_0;
	config.ledc_timer = LEDC_TIMER_0;
	config.pin_d0 = Y2_GPIO_NUM;
	config.pin_d1 = Y3_GPIO_NUM;
	config.pin_d2 = Y4_GPIO_NUM;
	config.pin_d3 = Y5_GPIO_NUM;
	config.pin_d4 = Y6_GPIO_NUM;
	config.pin_d5 = Y7_GPIO_NUM;
	config.pin_d6 = Y8_GPIO_NUM;
	config.pin_d7 = Y9_GPIO_NUM;
	config.pin_xclk = XCLK_GPIO_NUM;
	config.pin_pclk = PCLK_GPIO_NUM;
	config.pin_vsync = VSYNC_GPIO_NUM;
	config.pin_href = HREF_GPIO_NUM;
	config.pin_sscb_sda = SIOD_GPIO_NUM;
	config.pin_sscb_scl = SIOC_GPIO_NUM;
	config.pin_pwdn = PWDN_GPIO_NUM;
	config.pin_reset = RESET_GPIO_NUM;
	config.xclk_freq_hz = 20000000;
	config.pixel_format = PIXFORMAT_JPEG;

	if (psramFound())
	{
		// config.frame_size = FRAMESIZE_UXGA;
		config.frame_size = FRAMESIZE_QVGA;
		config.jpeg_quality = 10;
		config.fb_count = 2;
	}
	else
	{
		config.frame_size = FRAMESIZE_SVGA;
		config.jpeg_quality = 12;
		config.fb_count = 1;
	}
	// Camera init
	esp_err_t err = esp_camera_init(&config);
	if (err != ESP_OK)
	{
		Serial.printf("Camera init failed with error 0x%x", err);
		ESP.restart();
	}

	connectAWS();
}

void loop() 
{
  pushButton();
  client.loop();
  delay(1);
}
