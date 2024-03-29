#include <EEPROM.h>
#include "HX711.h"
#include "Adafruit_VL53L0X.h"

//*********** Load Cell

#define DT A1     //Load cell data pin (DT)
#define SCK A0    //Load cell clock pin (SCK)

//Load cell HX711
HX711 balanza(DT, SCK);

//Aux variables
long medidaRaw = 0;
long medida = 0;
long medidaEEPROM = 0;
long medidaCero = 8465393;
long adcActual = 0;
double peso = 0.0;
double pesoEEPROM = 0;
int muestras = 15;
double ADC_Entrada[15];
unsigned int contadorProducto = 0;
bool producto = false;


//********* Tray detection and traffic

//I2C Address
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOXt_ADDRESS 0x32

//Set the pins to shutdown
#define SHT_LOX1 7
#define SHT_LOX2 6
#define SHT_LOXt 5

//Objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X loxt = Adafruit_VL53L0X();

//Variables to hold the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measuret;

unsigned int trayDetection = 0;
unsigned int tray = 0;
bool transit = false;

unsigned int dist1 = 0;
unsigned int dist2 = 0;
unsigned int distt = 0;



//SETUP
void setup() {

  Serial.begin(115200);

  //HAND DETECTION BAR
  setOutputsLow();
  setAddress();

  //WEIGHT
  setCero();

}

//LOOP

void loop() {
  //CONTROL
  serialCommandReceive(); // for debugging purposes // 'e' for reset 0 and 'r'for reset eeprom and new fresh begin in the next restart

  //HAND BAR
  read_hand_bar();

  //WEIGHT
  measureWeight();
  comprobacionParametros();
}

//***********************FUNCTIONS******************************

long setCero() {
  medidaRaw = balanza.read_average(100);

  for (int i = 0; i < muestras; i++) {
    ADC_Entrada[i] = 0;
  }
  EEPROM.put(0x01, medidaRaw);
  EEPROM.put(0x00, 0x02);
  return medidaRaw;
}
///***********************************************************
void putEEPROM() {
  EEPROM.put(0x01, medidaRaw);
}
///***********************************************************
long getEEPROM() {
  EEPROM.get(0x01, medidaRaw);
  return medidaRaw;
}
///***********************************************************
void clearEEPROM() {
  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0xFE);
  }
}
///***********************************************************
void peso2EEPROM() {
  EEPROM.put(0x06, peso);
}

///***********************************************************
void setCeroManual(long ADC_Manual) {
  medidaRaw = ADC_Manual;
  putEEPROM();
}
///***********************************************************
void setCeroManualFast(long ADC_Manual) {
  medidaRaw = ADC_Manual;
  //putEEPROM();
}
///***********************************************************
void setValor(double nuevaMuestra) {
  for (int i = 0; i < muestras; i++) {
    ADC_Entrada[i] = nuevaMuestra;
  }
}
///***********************************************************
double medidaNueva() {
  double aux = balanza.read_average(1) - medidaRaw;
  //Serial.println(aux);
  return aux;
}
///***********************************************************
void agregarMedida() {

  //double nuevaMuestra = medidaNueva;
  //Serial.println("*******************");
  for (int i = 0; i < muestras - 1; i++) {
    ADC_Entrada[i] = ADC_Entrada[i + 1];
    //Serial.print(ADC_Entrada[i]);
    //Serial.print("|");
  }
  //Serial.println();
  //Serial.println("*******************");
  ADC_Entrada[muestras - 1] = medidaNueva();

}
///***********************************************************
double promedioMedida() {

  //double nuevaMuestra = medidaNueva;
  double suma = 0.0;

  for (int i = 0; i < muestras; i++) {
    suma = suma + ADC_Entrada[i];
  }
  suma =  suma / (double) muestras;
  //Serial.println(suma);
  return (double) suma;

}
///***********************************************************
double adctoKg(double promedioADC) {

  double pesoADC = -0.0193 * promedioADC + 1.19;
  pesoADC = pesoADC / 1000;
  return pesoADC;

}
///***********************************************************
void initEEPROMData() {
  if (EEPROM.read(0) == 0xFE || EEPROM.read(0) == 0x00) {
    Serial.print("Equipo recien inicializado, cargando ADC... Valor:");
    setCero();
    putEEPROM();
    EEPROM.write(0, 0x02); // cualquier numero distintos a los condicionados
    Serial.println(medidaRaw);
    peso2EEPROM();
  } else {
    Serial.print("Iniciando equipo, medida ADC-EEPROM: ");
    medidaRaw = getEEPROM();
    Serial.println(medidaRaw);
    EEPROM.write(0, 0x02);
  }

  EEPROM.get(0x06, pesoEEPROM);
}


///***********************************************************
void serialCommandReceive() {

  if (Serial.available() > 0) {

    byte inByte = Serial.read();
    // read the incoming byte:
    if (inByte == 'e') { //coloca en cero la eeprom
      clearEEPROM();
      setCero();
      pesoEEPROM = 0;
      putEEPROM();
      peso = 0;
      peso2EEPROM();
      pesoEEPROM = 0;
    }
    if (inByte == 'r') { //Reinicializa la eeprom (como si nunca se hubiera usado
      clearEEPROM();
      EEPROM.write(0, 0xFE);
      delay(100);
      Serial.println("EEPROM reinicializado");

    }

    if (inByte == 'k') { //Reinicializa la eeprom (como "r") y reinicia el MCU
      clearEEPROM();
      EEPROM.write(0, 0xFE);
      delay(100);
      Serial.println("EEPROM reinicializado");
      asm volatile ("  jmp 0");

    }
  }
}
///***********************************************************

void ajusteCeroEEPROM() {
  double pesoAux = 0;
  agregarMedida();
  peso = promedioMedida();
  peso = adctoKg(peso);


}
///***************************************************************
double measureWeight() {

  //pesoPID = (double)peso;
  agregarMedida();
  adcActual = promedioMedida();
  //Serial.println(adcActual);
  //adcActual = adcActual + medidaRaw;
  peso = adctoKg(adcActual);
  //Serial.println(peso, 3);
  return peso;

}
///*******************************************
void comprobacionParametros() {
  bool graph = false;
  if (!graph)
  {
    Serial.println();
    Serial.print("peso: ");
    Serial.print(peso, 3);
    Serial.print(" | ");
    Serial.print("pesoEEPROM: ");
    Serial.print(pesoEEPROM, 3);
    Serial.print(" | ");
    Serial.print("Distancia 1: ");
    Serial.print(dist1);
    Serial.print(" | ");
    Serial.print("Distancia 2: ");
    Serial.print(dist2);
    Serial.print(" | ");
    Serial.print("Distancia t: ");
    Serial.print(distt);
    Serial.print(" | ");
    Serial.print("Bandeja: ");
    Serial.print(tray);
    Serial.print(" | ");
    Serial.print("Transito: ");
    Serial.print(transit);
    Serial.print(" | ");

  }
  else
  {
    Serial.println(peso * 1000);
  }
}
///************************************************************



//********************* TRAFFIC and tray


//FUNCTIONS

void setOutputsLow() {
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOXt, OUTPUT);

  //Serial.println("Shutdown pins inited...");

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOXt, LOW);

  // Serial.println("All devices in reset mode...(pins are low)");
}

void setAddress() {
  //All reset
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOXt, LOW);

  delay(10);

  //All unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOXt, HIGH);

  delay(10);

  //Activating LOX1 and reseting the other devices
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOXt, LOW);

  //Initing LOX1
  if (!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot VL53L0X 1"));
    while (1);
  }
  delay(10);

  //Activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //Initing LOX2
  if (!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot VL53L0X 2"));
    while (1);
  }
  delay(10);

  //Activating LOXt
  digitalWrite(SHT_LOXt, HIGH);
  delay(10);

  //Initing LOXt
  if (!loxt.begin(LOXt_ADDRESS)) {
    Serial.println(F("Failed to boot VL53L0X for traffic"));
    while (1);
  }

  // Serial.println("All addresses set!");
}

void read_hand_bar() {


  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!
  loxt.rangingTest(&measuret, false); // pass in 'true' to get debug data printout!

  // print sensor one reading
  /*
    Serial.print("1: ");
    if (measure1.RangeStatus != 4) {    // if not out of range
    Serial.print(measure1.RangeMilliMeter);
    } else {
    Serial.print("Out of range");
    }

    Serial.println("  |  ");

    // print sensor two reading
    Serial.print("2: ");
    if (measure2.RangeStatus != 4) {
    Serial.print(measure2.RangeMilliMeter);
    } else {
    Serial.print("Out of range");
    }

    Serial.println("  |  ");

    // print sensor traffic reading
    Serial.print("t: ");
    if (measuret.RangeStatus != 4) {
    Serial.print(measuret.RangeMilliMeter);
    } else {
    Serial.print("Out of range");
    }
  */
  // Serial.println();

  dist1 = measure1.RangeMilliMeter;
  dist2 = measure2.RangeMilliMeter;
  distt = measuret.RangeMilliMeter;

  if (dist1 < 410 || dist2 < 410) {
    //Serial.println("****************BANDEJA 1");
    if (trayDetection != 1 || trayDetection == 0) trayDetection = 1;
    else if (trayDetection == 1) tray = 1;
  }

  else if (dist1 < 820 || dist2 < 820) {
    //Serial.println("****************BANDEJA 2");
    if (trayDetection != 2 || trayDetection == 0) trayDetection = 2;
    else if (trayDetection == 2) tray = 2;
  }

  else if (dist1 < 1050 || dist2 < 1050) {
    //Serial.println("****************BANDEJA 3");
    if (trayDetection != 3 || trayDetection == 0) trayDetection = 3;
    else if (trayDetection == 3) tray = 3;
  }
  else {
    trayDetection = 0;
    tray = 0;
  }

  if (distt < 500) {
    transit = true;
  }
  else transit = false;

  //delay(500);
}
