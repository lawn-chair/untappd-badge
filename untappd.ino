#include <SPI.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <Ticker.h>

#define FS_NO_GLOBALS
#include <FS.h>

#include <JPEGDecoder.h>
#include "ArduinoJson-v5.13.2.h"

//#include <SSD_13XX.h>
#include <TFT_eSPI.h>

#include <PubSubClient.h>

#include "secrets.h" // For AWS_ENDPOINT

void AWScallback(char* topic, byte* payload, unsigned int length);

uint32_t last_checkin = 0; 
bool g_flag_poll = 1;
bool g_badge_count = 0;

Ticker poller;

const char mqtt_server[] = AWS_ENDPOINT;
WiFiClientSecure link;
PubSubClient aws(mqtt_server,8883,AWScallback,link);

TFT_eSPI tft = TFT_eSPI();

// Return the minimum of two values a and b
#define minimum(a,b)     (((a) < (b)) ? (a) : (b))

//====================================================================================
//   Opens the image file and prime the Jpeg decoder
//====================================================================================
void drawJpeg(const char *filename, int xpos, int ypos) {
  // Open the named file (the Jpeg decoder library will close it after rendering image)
  fs::File jpegFile = SPIFFS.open( filename, "r");    // File handle reference for SPIFFS
 
  if ( !jpegFile ) {
    Serial.print("ERROR: File \""); Serial.print(filename); Serial.println ("\" not found!");
    return;
  }

  // Use one of the three following methods to initialise the decoder:
  boolean decoded = JpegDec.decodeFsFile(filename);  // or pass the filename (leading / distinguishes SPIFFS files)
                                   // Note: the filename can be a String or character array type
  if (decoded) {
    // render the image onto the screen at given coordinates
    if(JpegDec.width)
      jpegRender(xpos, ypos);
  }
  else {
    Serial.println(F("Jpeg file format not supported!"));
  }
}

//====================================================================================
//   Decode and render the Jpeg image onto the TFT screen
//====================================================================================
void jpegRender(int xpos, int ypos) {

  // retrieve infomration about the image
  uint16_t  *pImg;
  uint16_t mcu_w = JpegDec.MCUWidth;
  uint16_t mcu_h = JpegDec.MCUHeight;
  uint32_t max_x = JpegDec.width;
  uint32_t max_y = JpegDec.height;

  // Jpeg images are draw as a set of image block (tiles) called Minimum Coding Units (MCUs)
  // Typically these MCUs are 16x16 pixel blocks
  // Determine the width and height of the right and bottom edge image blocks
  uint32_t min_w = minimum(mcu_w, max_x % mcu_w);
  uint32_t min_h = minimum(mcu_h, max_y % mcu_h);

  // save the current image block size
  uint32_t win_w = mcu_w;
  uint32_t win_h = mcu_h;

  // record the current time so we can measure how long it takes to draw an image
  uint32_t drawTime = millis();

  // save the coordinate of the right and bottom edges to assist image cropping
  // to the screen size
  max_x += xpos;
  max_y += ypos;

  // read each MCU block until there are no more
#ifdef USE_SPI_BUFFER
  while( JpegDec.readSwappedBytes()){ // Swap byte order so the SPI buffer can be used
#else
  while ( JpegDec.read()) { // Normal byte order read
#endif
    // save a pointer to the image block
    pImg = JpegDec.pImage;

    // calculate where the image block should be drawn on the screen
    int mcu_x = JpegDec.MCUx * mcu_w + xpos;
    int mcu_y = JpegDec.MCUy * mcu_h + ypos;

    // check if the image block size needs to be changed for the right and bottom edges
    if (mcu_x + mcu_w <= max_x) win_w = mcu_w;
    else win_w = min_w;
    if (mcu_y + mcu_h <= max_y) win_h = mcu_h;
    else win_h = min_h;

    // calculate how many pixels must be drawn
    uint32_t mcu_pixels = win_w * win_h;

    // draw image MCU block only if it will fit on the screen
    if ( ( mcu_x + win_w) <= tft.width() && ( mcu_y + win_h) <= tft.height())
	{
#ifdef USE_SPI_BUFFER
      // Now set a MCU bounding window on the TFT to push pixels into (x, y, x + width - 1, y + height - 1)
      tft.setWindow(mcu_x, mcu_y, mcu_x + win_w - 1, mcu_y + win_h - 1);
      // Write all MCU pixels to the TFT window
      uint8_t *pImg8 = (uint8_t*)pImg;     // Convert 16 bit pointer to an 8 bit pointer
      tft.pushColors(pImg8, mcu_pixels*2); // Send bytes via 64 byte SPI port buffer
#else
      // Now set a MCU bounding window on the TFT to push pixels into (x, y, x + width - 1, y + height - 1)
      tft.setAddrWindow(mcu_x, mcu_y, mcu_x + win_w - 1, mcu_y + win_h - 1);
      // Write all MCU pixels to the TFT window
      while (mcu_pixels--) tft.pushColor(*pImg++);

#endif
    }

    else if ( ( mcu_y + win_h) >= tft.height()) JpegDec.abort();

  }

  // calculate how long it took to draw the image
  drawTime = millis() - drawTime; // Calculate the time it took

  // print the results to the Serial port
  Serial.print  ("Total render time was    : "); Serial.print(drawTime); Serial.println(" ms");
  Serial.println("=====================================");

}

void timer_callback()
{
  g_flag_poll = 1;
}

void AWScallback(char* topic, byte* payload, unsigned int length) {
  const size_t bufferSize = JSON_OBJECT_SIZE(3) + 30;
  DynamicJsonBuffer jsonBuffer(bufferSize);

  JsonObject& root = jsonBuffer.parseObject(payload);
  if(root.containsKey("init"))
  {
    last_checkin = root["init"];
  }
  else if(root.containsKey("name"))
  {
    displayBadge(root["name"], root["image"]);
    g_badge_count = 1;
  }
  if(root.containsKey("id"))
  {
    if(root["id"] > last_checkin)
    {
      last_checkin = root["id"];
    }
  }
  jsonBuffer.clear();
}

void aws_reconnect()
{
  while(!aws.connected())
  {
    Serial.print(F("Connecting to AWS..."));
    if(aws.connect("Untappd"))
    {
      Serial.println(F("connected"));
      aws.subscribe("untappdReply");
    }
    else
    {
      Serial.print(F("failed, rc="));
      Serial.print(aws.state());
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(250000);
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  if(!SPIFFS.begin())
  {
    Serial.print(F("SPIFFS Error"));
    while(1) {};
  }

  Serial.print(F("heap: ")); Serial.println(ESP.getFreeHeap());

  WiFi.begin(AP, AP_PASSWORD);

  delay(100);
  tft.print(F("Connecting"));
  while (WiFi.status() != WL_CONNECTED)
  { 
    delay(500);
    tft.print(".");
  }
  tft.println();

  tft.print(F("Connected, IP address: "));
  tft.println(WiFi.localIP());

  fs::File cert = SPIFFS.open(F("/cert.der"), "r");
  if(cert)
  {
    link.loadCertificate(cert);
  }

  fs::File key = SPIFFS.open(F("/private.der"), "r");
  if(key)
  {
    link.loadPrivateKey(key);
  }
  // necessary?
  cert.close();
  key.close();

  delay(1500);

  tft.fillScreen(TFT_BLACK);
  drawJpeg("/avatar.jpg", 14, 0);
  tft.drawString(F("lexuschris"), 8, 104, 4);

  poller.attach(90, timer_callback);

  Serial.print(F("heap: ")); Serial.println(ESP.getFreeHeap());
}

void downloadAndSave(const char *filename)
{
  // download and save, then display.  Not enough memory :(
  Serial.println(F("getting image"));
  fs::File f = SPIFFS.open(filename, "w");

  uint8_t buffer[64];
  HTTPClient client;
  client.begin(String(F("https://untappd.akamaized.net/badges")) + filename, "");
  int response = client.GET();
  if(response != 200)
  {
    Serial.print(F("HTTP GET FAILED."));
  } else {
    WiFiClient *stream = client.getStreamPtr();
    int len = client.getSize();
    while(client.connected() && (len > 0 || len == -1))
    {
      size_t avail = stream->available();
      if(avail)
      {
        int c = stream->readBytes(buffer, ((avail > sizeof(buffer)) ? sizeof(buffer) : avail));
        f.write(buffer, c);

        if(len > 0)
        {
          len -= c;
        }
        delay(1);
      }
    }
  }

  f.close();
  client.end();
}

void displayBadge(const char *name, const char *filename)
{
  Serial.print(F("displayBadge"));
  Serial.println(filename);

  tft.fillScreen(TFT_BLACK);
  tft.fillScreen(TFT_WHITE);
  delay(500);

  if(!SPIFFS.exists(filename))
  {
   drawJpeg("/bdg_default_sm.jpg", 19, 2);
  }
  else
  {
    drawJpeg(filename, 19, 2);
  }

  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setCursor(0, 94, 2);
  tft.print(name);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  delay(5000);

}

void loop() {
  if(!aws.connected())
  {
    aws_reconnect();
  }

  if(last_checkin == 0)
  {
    Serial.println("Sending message");
    aws.publish("untappd", "{\"type\": \"init\", \"user\": \"lexuschris\"}");
    last_checkin = 1;
  }

  aws.loop();
  
  if(g_flag_poll && last_checkin > 1)
  {
    g_flag_poll = 0;
    String json = String(F("{\"type\": \"poll\", \"user\": \"lexuschris\", \"since\": \"")) + last_checkin + "\"}"; 
    aws.publish("untappd", json.c_str());
  }

  if(g_badge_count)
  {
    g_badge_count = 0;
    tft.fillScreen(TFT_BLACK);
    drawJpeg("/avatar.jpg", 14, 0);
    tft.drawString(F("lexuschris"), 8, 104, 4);
  }

  //Serial.print(F("heap: ")); Serial.println(ESP.getFreeHeap());
}