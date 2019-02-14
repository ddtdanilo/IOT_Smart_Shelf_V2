#include "Adafruit_VL53L0X.h"

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

void setup() {

  //SERIAL
  Serial.begin(115200);
  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }

  //HAND DETECTION BAR
  setOutputsLow();
  setAddress();

  //WEIGHT
  //...

}

void loop() {

  //HAND BAR
  read_hand_bar();

  //WEIGHT
  //...

}


//FUNCTIONS

void setOutputsLow() {
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOXt, OUTPUT);

  Serial.println("Shutdown pins inited...");

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOXt, LOW);

  Serial.println("All devices in reset mode...(pins are low)");
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

  Serial.println("All addresses set!");
}

void read_hand_bar() {


  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!
  loxt.rangingTest(&measuret, false); // pass in 'true' to get debug data printout!

  // print sensor one reading
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

  Serial.println();



  if (measure1.RangeMilliMeter < 410 || measure2.RangeMilliMeter < 410) {
    //Serial.println("****************BANDEJA 1");
    if (trayDetection != 1 || trayDetection == 0) trayDetection = 1;
    else if(trayDetection == 1) Serial.println("*****BANDEJA 1");
  }

  else if (measure1.RangeMilliMeter < 820 || measure2.RangeMilliMeter < 820) {
    //Serial.println("****************BANDEJA 2");
    if (trayDetection != 2 || trayDetection == 0) trayDetection = 2;
    else if (trayDetection == 2) Serial.println("*****BANDEJA 2");
  }

  else if (measure1.RangeMilliMeter < 1050 || measure2.RangeMilliMeter < 1050) {
    //Serial.println("****************BANDEJA 3");
    if (trayDetection != 3 || trayDetection == 0) trayDetection = 3;
    else if (trayDetection == 3) Serial.println("*****BANDEJA 3");
  }
  else trayDetection = 0;







  if (measuret.RangeMilliMeter < 500) {
    Serial.println("****************TRANSITO");
  }

  //delay(500);
}
