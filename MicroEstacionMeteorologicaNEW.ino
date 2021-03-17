#include <Cloudino.h>
#include "DHT.h"
#include "Adafruit_CCS811.h"
Cloudino cdino;
Adafruit_CCS811 ccs;

/////////DHT22///////////////
#define DHTPIN 8  
#define DHTTYPE DHT22 
DHT dht(DHTPIN, DHTTYPE);
//////////////////////////////


void getSensor()
{
  
/////////DHT22///////////////
delay(2000);
float temperature =getTemp("c");
float farenheit = getTemp("f");
float kelvin =getTemp("k");
float humedad =getTemp("h");
cdino.post("celsius",String(temperature));
cdino.post("farenheit",String(farenheit));
cdino.post("kelvin",String(kelvin));
cdino.post("humedad",String(humedad));
///////////////////////////////////ccs811///////////////////////////
if(ccs.available()){


    if(!ccs.readData()){

float ccsTEMP = ccs.calculateTemperature();
float co2 = ccs.geteCO2();
float TVOC = ccs.getTVOC();
cdino.post("co2",String(co2));
cdino.post("TVOC",String(TVOC));
cdino.post("ccsTEMP",String(ccsTEMP));
    }
    else{
      Serial.println("Sensor read ERROR!");
      ccs.readData();
    }
  }
/////////////////////////////////UV////////////////////////////////////
 int sensorvalue = analogRead(A2);
 float UVindex = ((sensorvalue*(5.0/1023.0))/.1);
 cdino.post("UVindex",String(UVindex));
///////////////////////////////////////////////////////////////////////

}

float getTemp(String req)
{

  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);


  float hif = dht.computeHeatIndex(f, h);
  float hic = dht.computeHeatIndex(t, h, false);
  if (isnan(h) || isnan(t) || isnan(f)) {
    return;
  }
  // Compute heat index in Kelvin 
  float k = t + 273.15;
  if(req =="c"){
    return t;//return Cilsus
  }else if(req =="f"){
    return f;// return Fahrenheit
  }else if(req =="h"){
    return h;// return humidity
  }else if(req =="hif"){
    return hif;// return heat index in Fahrenheit
  }else if(req =="hic"){
    return hic;// return heat index in Cilsus
  }else if(req =="k"){
    return k;// return temprature in Kelvin
  }else{
    return 0.000;// if no reqest found, retun 0.000
  }

}

void setup() {
  cdino.setInterval(5000,getSensor);

  
  if(!ccs.begin()){
    while(1);
  }
  //calibrate temperature sensor
  while(!ccs.available());
  float temp = ccs.calculateTemperature();
  ccs.setTempOffset(temp - 25.0);

  dht.begin();
  cdino.begin();
}



void loop() {
  cdino.loop();
  
}
