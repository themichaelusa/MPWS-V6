#include <DHT.h>
#include <OneWire.h>

#define DHTPIN 7  //Digital Pin 2
#define DHTTYPE DHT11 
DHT dht(DHTPIN, DHTTYPE);

OneWire  ds(10); 

int sensorPinMoist = A0; //Grove Moisture Sensor (A0)
int waterPump = 2; //5v Aquarium Pump (2)
int solenoidPin = 4; //Solenoid Valve (OUTPUT) (4)

int sensorValueGroveMoisture = 0;

//DHT 11 Vars
float h = dht.readHumidity();
// Read temperature as Celsius (the default)
float t = dht.readTemperature();

//Soil Temp Vars
float celsius, fahrenheit;
 
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode(9, OUTPUT); //piezo beeper
  pinMode(waterPump, OUTPUT); //5v water pump
  pinMode(solenoidPin, OUTPUT); //valve

  dht.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  
   //grove moisture 
  sensorValueGroveMoisture = digitalRead(sensorPinMoist);    
  delay(1000);          
  Serial.print("Moisture Level =   " );                       
  Serial.println(sensorValueGroveMoisture);    

//main water deployment statement: t is air temp,h is humididty, fahr is soil temp

 if (sensorValueGroveMoisture < 350 || t>105 ||fahrenheit>90 || (h<5 && t>105)) {
 do { 
   beep(50),beep(50),beep(50),beep(50),beep(50),beep(50);
   delay(1000);
  if (sensorValueGroveMoisture > 350) {break;}
  }while (sensorValueGroveMoisture < 350); 
 digitalWrite(solenoidPin, HIGH);   //solenoid on
 digitalWrite(waterPump, HIGH);   //pump on                  
  }
 else {
  digitalWrite(9, LOW);
  digitalWrite(waterPump, LOW);   //pump off  
  digitalWrite(solenoidPin, LOW);    //Solenoid off
  }
  
//DHT AIR TEMP SENSOR//////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print(" *C ");
 
// WATER TEMP SENSOR////////////////////////////////////////////////////////////////////////
 byte i;
 byte present = 0;
 byte type_s;
 byte data[12];
 byte addr[8];
 float celsius, fahrenheit;

  if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
    return;
  }
  
  Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }

 if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }
 Serial.println();

  // the first ROM byte indicates which chip

  switch (addr[0]) {
    case 0x10:
    Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
      case 0x28:
      Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
      case 0x22:
      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
     default:
      Serial.println("Device is not a DS18x20 family device.");
      return;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(0x44);        // start conversion, use ds.write(0x44,1) with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  Serial.print("  Data = ");
  Serial.print(present, HEX);
  Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }

  Serial.print(" CRC=");
  Serial.print(OneWire::crc8(data, 8), HEX);
  Serial.println();

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.

  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }

  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
  Serial.print("  Temperature = ");
  Serial.print(celsius);
  Serial.print(" Celsius, ");
  Serial.print(fahrenheit);
  Serial.println(" Fahrenheit");
}

void beep(unsigned char delayms){
  analogWrite(9, 20);      // accepts 1-254
  delay(delayms);          // waits
  analogWrite(9, 0);       // 0 means off, bob
  delay(delayms);          // waits for delayms ms   
}  

