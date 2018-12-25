





#include <TinyGPS++.h>
#include <SoftwareSerial.h>



#include <EEPROM.h> 
#include <ESP8266WiFi.h>

#include <ESP8266HTTPClient.h>

#include <ArduinoOTA.h> // Библиотека для OTA-прошивки
 
#include "math.h" 




//wifii&server




const char* ssid     = "Gansta-Paradise.gps"; 
const char* password = ""; 
  
const char* host = "www.gansta-paradise-forum.ru"; 




int interval = 0;


#include <OneWire.h> 


#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif

uint8_t bell[8]  = {0x4,0xe,0xe,0xe,0x1f,0x0,0x4};
uint8_t note[8]  = {0x2,0x3,0x2,0xe,0x1e,0xc,0x0};
uint8_t clock[8] = {0x0,0xe,0x15,0x17,0x11,0xe,0x0};
uint8_t heart[8] = {0x0,0xa,0x1f,0x1f,0xe,0x4,0x0};
uint8_t duck[8]  = {0x0,0xc,0x1d,0xf,0xf,0x6,0x0};
uint8_t check[8] = {0x0,0x1,0x3,0x16,0x1c,0x8,0x0};
uint8_t cross[8] = {0x0,0x1b,0xe,0x4,0xe,0x1b,0x0};
uint8_t retarrow[8] = {  0x1,0x1,0x5,0x9,0x1f,0x8,0x4};
  
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
/*
   This sample code demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = D5, TXPin = D6;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);






///unsigned char meteringTime  = 0;       // Время замера

///unsigned long startMillis   = 0; // Начало отсчета
unsigned long startITOW     = 0; // Начало отсчета из GPS
///unsigned long currentMillis = 0; // Текущее время

float speedKM = 0;  //////скорость 


char gpsSpeed[3];       // Буфер для строки с скоростью
int gpsSpeedKm = 0;     // Скорость в км/ч
bool start = false;     // Старт замера
long startMillis = 0;   // Начало отсчета
long currentMillis = 0; // Текущее время
float meteringTime = 0; //


struct Metering
{
  float accel30;
  float accel60;
  float accel100;
};
Metering metering;

int startzamer = 0;


///int tonePin = D7;


double privateKey ;

double privateKeyL ;

float privateKeyS ;

float privateKey30 ;  
float privateKey60 ;
float privateKey100 ;

String Latitude;

String Longitude;
int value = 0;



void setup()
{

  ///// pinMode(D5, INPUT_PULLUP);
      pinMode(D7,INPUT_PULLUP);
  Serial.begin(115200);
  ss.begin(GPSBaud);



  Serial.println(TinyGPSPlus::libraryVersion());
 
  Serial.println();
  Serial.println(F("Sats HDOP Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
  Serial.println(F("          (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail"));
  Serial.println(F("---------------------------------------------------------------------------------------------------------------------------------------"));


  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  
  //lcd.createChar(0, bell);
  //lcd.createChar(1, note);
  lcd.createChar(2, clock);
 lcd.createChar(3, heart);
  lcd.createChar(4, duck);
  lcd.createChar(5, check);
  lcd.createChar(6, cross);
  lcd.createChar(7, retarrow);
  lcd.home();
  
  lcd.print("RAZGONOMETR...");
  lcd.setCursor(0, 1);
  
  lcd.printByte(3);
  lcd.print("GANSTA-PARADISE");
  delay(5000);
  //displayKeyCodes();

lcd.clear();


  
}

void loop()
{
  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printInt(gps.hdop.value(), gps.hdop.isValid(), 5);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.value()) : "*** ", 6);

    lcd.home();
  
  lcd.print(gps.location.lat(),5);
  lcd.printByte(5);
  lcd.print(gps.location.lng(),5);

 lcd.setCursor(0, 1);
  lcd.print(gps.speed.kmph());
  lcd.printByte(4);
  lcd.print("SPEED");
  lcd.printByte(2);
  lcd.print(interval);
  smartDelay(30);
  speedKM = gps.speed.kmph();///////////////скорость..........вжжжжжж............
  
privateKeyS = gps.speed.kmph();
  unsigned long distanceKmToLondon =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON) / 1000;
  printInt(distanceKmToLondon, gps.location.isValid(), 9);

  double courseToLondon =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON);

  printFloat(courseToLondon, gps.location.isValid(), 7, 2);

  const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToLondon);

  printStr(gps.location.isValid() ? cardinalToLondon : "*** ", 6);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  Serial.println();
  
  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));

    
interval++;

zamer();

if(digitalRead(D7)==LOW){
  lcd.clear();
  startzamer++;
   lcd.setCursor(0, 1);
     lcd.setCursor(0, 0);
 lcd.print("Rezultat");
 
 smartDelay(1000);
  
  
          
          }


  

/// Serial.println(interval);

if(interval>300&&gps.location.lat()>0){
 /// delay(100);
sendtowebGPS();
smartDelay(100);
interval = 0;
lcd.clear();
lcd.setCursor(0, 0);
lcd.print("GPS-SEND");
smartDelay(1000);
lcd.setCursor(0, 1);
///lcd.print( currentMillis);
  lcd.print(gps.location.lat(),5);
  lcd.printByte(5);
  lcd.print(gps.location.lng(),5);
smartDelay(10000);
lcd.clear();


}


////////////posl scob
if(interval==100){
  
 lcd.clear();
 }

if(interval==10){
  
 lcd.clear();
 }
 
}



// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
      ESP.wdtDisable(); 

  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
    
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}


void zamer(){


 currentMillis = millis();

 gpsSpeedKm = gps.speed.kmph();

 
   if (gpsSpeedKm > 3 ) {
      // Если это был старт
      ////lcd.clear();
      lcd.printByte(6);
      if (!start) {
        start = true;
        startMillis = millis();
      }
      meteringTime = (float)(currentMillis - startMillis) / 1000; // Время замера
      // Результаты замера
      if (0.0 == metering.accel30 && gpsSpeedKm >= 30) {
        metering.accel30 = meteringTime; // Разгон до 30км/ч
      }
      else if (0.0 == metering.accel60 && gpsSpeedKm >= 60) {
        metering.accel60 = meteringTime; // Разгон до 60км/ч
      }
      else if (0.0 == metering.accel100 && gpsSpeedKm >= 100) {
        metering.accel100 = meteringTime; // Разгон до 100км/ч
      }
    }
    else if (start && gpsSpeedKm<3) { // Если остановилисьelse if (start && 0 == gpsSpeedKm)
      start = false;
    }

    if ( startzamer > 0 &&meteringTime > 1 ) {
    lcd.clear();
lcd.setCursor(0, 0);
lcd.print("ZAMER");
lcd.printByte(5);
lcd.print( metering.accel30);
lcd.printByte(5);
lcd.print( metering.accel60);
smartDelay(100);
lcd.setCursor(0, 1);
lcd.print( metering.accel100);
smartDelay(10000);    
 
  metering.accel30=0.0;
  metering.accel60=0.0;
  metering.accel100=0.0; 
 meteringTime=0;
start = false;
interval = 0;
startzamer = 0;

}else if  ( startzamer > 0 &&meteringTime <1 ) {
 lcd.clear();
lcd.setCursor(0, 0);
lcd.print("NO RESULTS");
smartDelay(1000);
interval = 0;
startzamer = 0;
 start = false;
  
}
privateKey30 = metering.accel30;
privateKey60 = metering.accel60;
privateKey100 = metering.accel100;
  }
void sendtowebGPS(){
 ///Serial.begin(115200);
 ESP.wdtDisable();
  smartDelay(4000);
  WiFi.begin(ssid, password); 
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  } 
  Serial.println("Connected to the WiFi network");
   Serial.println(Latitude);
    Serial.println(Latitude); 
   delay(1000);   

  ++value;

    Serial.print("connecting to ");
    Serial.println(host);

    // Use WiFiClient class to create TCP connections
    WiFiClient client;
    const int httpPort = 80;
    if (!client.connect(host, httpPort)) {
        Serial.println("connection failed");
        return;
    }

    // We now create a URI for the request
    String url = "/gps.php?pak0=";
   /// url += streamId;
   /// url += "?private_key=";
  Latitude = String(gps.location.lat(),5);
  Longitude = String(gps.location.lng(),5);
      url +=Latitude;
    url += "&pak1=";
    ///url += gps.location.lng(),5;
    url += Longitude;
    url += "&pak2=";
    url += privateKeyS;
     url += "&pak3=";
    url += privateKey30;
     url += "&pak4=";
    url += privateKey60;
     url += "&pak5=";
    url += privateKey100;

    Serial.print("Requesting URL: ");
    Serial.println(url);

    // This will send the request to the server
    client.print(String("GET ") + url + " HTTP/1.1\r\n" +
                 "Host: " + host + "\r\n" +
                 "Connection: close\r\n\r\n");
    unsigned long timeout = millis();
    while (client.available() == 0) {
        if (millis() - timeout > 5000) {
            Serial.println(">>> Client Timeout !");
            client.stop();
             ESP.wdtDisable(); 
            return;
        }
    }

    // Read all the lines of the reply from server and print them to Serial
    while(client.available()) {
        String line = client.readStringUntil('\r');
        Serial.print(line);
         ESP.wdtDisable(); 
    }

    Serial.println();
    Serial.println("closing connection");

     ESP.wdtDisable(); 
   

Serial.println("srabotalo-web-GPS");

 ///ESP.reset();

  
}
