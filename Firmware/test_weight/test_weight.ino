
#include <EEPROM.h>
#include "HX711.h"

#define DT A1 // Pin analógico A1 para el pin DT del transmisor de celda de carga HX711
#define SCK A0 // Pin analógico A0 para el pin SCK del transmisor de celda de carga HX711

// Creación del objeto para el transmisor de celda de carga HX711
HX711 balanza(DT, SCK);
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




void setup() {

  Serial.begin(115200); // Comienzo de la comunicación con el monitor serie
  setCero();

}







///*************************************
///************** LOOP *****************
///*************************************

void loop() {

  serialCommandReceive(); // for debugging purposes // 'e' for reset 0 and 'r'for reset eeprom and new fresh begin in the next restart
  measureWeight();
  comprobacionParametros();
}
















// ***************************************************************************

// *************************     FUNCIONES     *******************************

// ***************************************************************************
///***********************************************************

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

///********************************



///*************************************************




///*******************************************
void comprobacionParametros()
{


  bool graph = true;
  if (!graph)
  {
    Serial.println();
    Serial.print("peso: ");
    Serial.print(peso, 3);
    Serial.print(" | ");
    Serial.print("pesoEEPROM: ");
    Serial.print(pesoEEPROM, 3);
    Serial.print(" | ");

  }

  else
  {
    Serial.println(peso * 1000);

  }



}


///************************************************************
