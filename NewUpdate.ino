#include <Cloudino.h>
#include "DHT.h"
Cloudino cdino;
#include <Wire.h>    // incluye libreria para interfaz I2C
#include <RTClib.h>   // incluye libreria para el manejo del modulo RTC

RTC_DS3231 rtc;     // crea objeto del tipo RTC_DS3231


///SD///
#include <SPI.h>    // incluye libreria interfaz SPI
#include <SD.h>     // incluye libreria para tarjetas SD

#define SSpin 10    // Slave Select en pin digital 10

File archivo;     // objeto archivo del tipo File




/////////DHT22///////////////
#define DHTPIN 8
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
////////////UV//////////////////
int UVOUT = A0; //Output from the sensor
int REF_3V3 = A1; //3.3V power on the Arduino board

int enable = 1;


////Sensor de humedad de Suelo///
const int sensorPin = A3;


///sensor de Viento///
#define analogPinForRV    6   // change to pins you the analog pins are using
#define analogPinForTMP   7

// to calibrate your sensor, put a glass over it, but the sensor should not be
// touching the desktop surface however.
// adjust the zeroWindAdjustment until your sensor reads about zero with the glass over it. 

const float zeroWindAdjustment =  .5; // negative numbers yield smaller wind speeds and vice versa.

int TMP_Therm_ADunits;  //temp termistor value from wind sensor
float RV_Wind_ADunits;    //RV output from wind sensor 
float RV_Wind_Volts;
unsigned long lastMillis;
int TempCtimes100;
float zeroWind_ADunits;
float zeroWind_volts;
float WindSpeed_MPH;





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
    return 0.0;
  }
  // Compute heat index in Kelvin
  float k = t + 273.15;
  if (req == "c") {
    return t;//return Cilsus
  } else if (req == "f") {
    return f;// return Fahrenheit
  } else if (req == "h") {
    return h;// return humidity
  } else if (req == "hif") {
    return hif;// return heat index in Fahrenheit
  } else if (req == "hic") {
    return hic;// return heat index in Cilsus
  } else if (req == "k") {
    return k;// return temprature in Kelvin
  } else {
    return 0.000;// if no reqest found, retun 0.000
  }

}



//Takes an average of readings on a given pin
//Returns the average
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0;
  for (int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;
  return (runningValue);
}


//The Arduino Map function but for floats
//From: http://forum.arduino.cc/index.php?topic=3922.0
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void DHTget() {
  float temperature = getTemp("c");
  float farenheit = getTemp("f");
  float kelvin = getTemp("k");
  float humedad = getTemp("h");
  cdino.post("celsius", String(temperature));
  cdino.post("farenheit", String(farenheit));
  cdino.post("kelvin", String(kelvin));
  cdino.post("humedad", String(humedad));
}

void UVget() {
  int uvLevel = averageAnalogRead(UVOUT);
  int refLevel = averageAnalogRead(REF_3V3);
  //Use the 3.3V power pin as a reference to get a very accurate output value from sensor
  float outputVoltage = 3.3 / refLevel * uvLevel;
  float uvIntensity = mapfloat(outputVoltage, 0.99, 2.8, 0.0, 15.0); //Convert the voltage to a UV intensity level

  cdino.post("uvIntensidad", String(uvIntensity));
}

void getSueloH(){
   int humedadS = analogRead(sensorPin);
   int hum = map(humedadS, 0, 1023, 0, 100);
   cdino.post("HumedadS", String(hum));
  
  }

void getSensor()
{
  
  delay(200);
   DHTget();
   UVget();
   getSueloH();
   getWind();
  DateTime fecha = rtc.now();
  if (fecha.hour() == 7 or fecha.hour() == 14 or fecha.hour() == 23 ) {
    while (enable == 1) {
      postSD();
      enable = 0;
    }
  } else {
    enable = 1;
  }

}
void getWind(){
  
    TMP_Therm_ADunits = analogRead(analogPinForTMP);
    RV_Wind_ADunits = analogRead(analogPinForRV);
    RV_Wind_Volts = (RV_Wind_ADunits *  0.0048828125);

    // these are all derived from regressions from raw data as such they depend on a lot of experimental factors
    // such as accuracy of temp sensors, and voltage at the actual wind sensor, (wire losses) which were unaccouted for.
    TempCtimes100 = (0.005 *((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits)) - (16.862 * (float)TMP_Therm_ADunits) + 9075.4;  

    zeroWind_ADunits = -0.0006*((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits) + 1.0727 * (float)TMP_Therm_ADunits + 47.172;  //  13.0C  553  482.39

    zeroWind_volts = (zeroWind_ADunits * 0.0048828125) - zeroWindAdjustment;  

    // This from a regression from data in the form of 
    // Vraw = V0 + b * WindSpeed ^ c
    // V0 is zero wind at a particular temperature
    // The constants b and c were determined by some Excel wrangling with the solver.
    
   WindSpeed_MPH =  pow(((RV_Wind_Volts - zeroWind_volts) /.2300) , 2.7265);   
   
  cdino.post("WindSpeed_MPH", String(WindSpeed_MPH));
 

}



void postSD() {

  archivo = SD.open("data.txt", FILE_WRITE);  // apertura para lectura/escritura de archivo prueba.txt
  if (archivo) {

    float temperature = getTemp("c");
    float humedad = getTemp("h");
    int uvLevel = averageAnalogRead(UVOUT);
    int refLevel = averageAnalogRead(REF_3V3);
    //Use the 3.3V power pin as a reference to get a very accurate output value from sensor
    float outputVoltage = 3.3 / refLevel * uvLevel;
    float uvIntensity = mapfloat(outputVoltage, 0.99, 2.8, 0.0, 15.0); //Convert the voltage to a UV intensity level

    TMP_Therm_ADunits = analogRead(analogPinForTMP);
    RV_Wind_ADunits = analogRead(analogPinForRV);
    RV_Wind_Volts = (RV_Wind_ADunits *  0.0048828125);

    // these are all derived from regressions from raw data as such they depend on a lot of experimental factors
    // such as accuracy of temp sensors, and voltage at the actual wind sensor, (wire losses) which were unaccouted for.
    TempCtimes100 = (0.005 *((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits)) - (16.862 * (float)TMP_Therm_ADunits) + 9075.4;  

    zeroWind_ADunits = -0.0006*((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits) + 1.0727 * (float)TMP_Therm_ADunits + 47.172;  //  13.0C  553  482.39

    zeroWind_volts = (zeroWind_ADunits * 0.0048828125) - zeroWindAdjustment;  

    // This from a regression from data in the form of 
    // Vraw = V0 + b * WindSpeed ^ c
    // V0 is zero wind at a particular temperature
    // The constants b and c were determined by some Excel wrangling with the solver.
    
   WindSpeed_MPH =  pow(((RV_Wind_Volts - zeroWind_volts) /.2300) , 2.7265);   

   int humedadS = analogRead(sensorPin);
   int hum = map(humedadS, 0, 1023, 0, 100);
    

    DateTime fecha = rtc.now(); // funcion que devuelve fecha y horario en formato
    // DateTime y asigna a variable fecha
    archivo.print(fecha.day());     // funcion que obtiene el dia de la fecha completa
    archivo.print("/");       // caracter barra como separador
    archivo.print(fecha.month());     // funcion que obtiene el mes de la fecha completa
    archivo.print("/");       // caracter barra como separador
    archivo.print(fecha.year());      // funcion que obtiene el año de la fecha completa
    archivo.print(", ");       // caracter espacio en blanco como separador
    archivo.print(fecha.hour());      // funcion que obtiene la hora de la fecha completa
    archivo.print(":");       // caracter dos puntos como separador
    archivo.print(fecha.minute());      // funcion que obtiene los minutos de la fecha completa
    archivo.print(":");       // caracter dos puntos como separador
    archivo.print(fecha.second());    // funcion que obtiene los segundos de la fecha completa
    archivo.print(", ");
    archivo.print(String(temperature)); // escritura de una linea de texto en archivo
    archivo.print(", ");
    archivo.print(String(humedad)); // escritura de una linea de texto en archivo
    archivo.print(", ");
    archivo.print(String(uvIntensity)); // escritura de una linea de texto en archivo
    archivo.print(", ");
    archivo.print(String(WindSpeed_MPH)); // escritura de una linea de texto en archivo
    archivo.print(", ");
    archivo.println(String(hum));
    archivo.close();        // cierre del archivo
    Serial.println("escritura correcta"); // texto de escritura correcta en monitor serie
  } else {
    Serial.println("error en apertura de prueba.txt");  // texto de falla en apertura de archivo
  }

}





void setup() {
  Serial.begin(9600);    // inicializa comunicacion serie a 9600 bps
  
  pinMode(UVOUT, INPUT);
  pinMode(REF_3V3, INPUT);


  cdino.setInterval(1000, getSensor);
  dht.begin();
  cdino.begin();

  if (!SD.begin(SSpin)) {     // inicializacion de tarjeta SD
    Serial.println("fallo en inicializacion !");// si falla se muestra texto correspondiente y
  }

  if (! rtc.begin()) {       // si falla la inicializacion del modulo
    Serial.println("Modulo RTC no encontrado !");  // muestra mensaje de error
    // bucle infinito que detiene ejecucion del programa
  }

}


void loop() {
  cdino.loop();

}
