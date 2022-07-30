/**************************************************
 * Testing:
 *  pulse flow sensor, ThrowIn Senor
 *  ina3221 3chan analog monitor I2C
 *  ads1115 4chan 16bit adc I2C
 *  hw-685 Current To Volts Converter
 *
 * Reading sensors
 *  gr-108 paddle wheel plastic 1-60lpm 1.2Mpa f=5.5*q q=lpm
 *  cf-b10 impeller brass 2-50lpm 1.75Mpa f=7.5*q-4
 *  g1/4 Pressure Transducer 80psi 5v
 *  1/4 pressure xdcr 200psi 4-20ma 12v
 *
 * Interfaces
 *  Current to voltage converter 0/4-20ma to 0-3.3/5/10v connected esp32
 *  ina3221 voltage monitor connected to 80psi 5v sensor and 4-20ma
 *
 * //Explanation of I2C address for INA3221:
 *   //INA3221_ADDR40_GND = 0b1000000, // A0 pin -> GND
 *   //INA3221_ADDR41_VCC = 0b1000001, // A0 pin -> VCC
 *   //INA3221_ADDR42_SDA = 0b1000010, // A0 pin -> SDA
 *   //INA3221_ADDR43_SCL = 0b1000011  // A0 pin -> SCL
 *
 *
 * working on pulse flow sensor
 * plastic sensor use the following calculation
 * Sensor Frequency (Hz) = 5.5 * Q (Liters/min)

 * Liters = Q * time elapsed (seconds) / 60 (seconds/minute)
 * Liters = (Frequency (Pulses/second) / 5.5) * time elapsed (seconds) / 60
 * Liters = Pulses / (5.5 * 60)
 * float liters = pulses;
 * liters /= 7.5;
 * liters /= 60.0;
 *
 *  both flow sensors work
 *  brass seems to have less back pressure
 *  both need to be calibrated
 *
 * **************************************
 * working on ina3221
 * tried 2 libs
 * dosen't seem to have the resolution
 * using CurrentGen or ThrowIn jumps in .4ma steps
 * may need to change shunt resistor to:
 * If I'm reading datasheet the max volts is 168mv across the shunt
 *  shut max volts / max current = resistor
 *           168mv / 20ma        = 8.4 ohm   is this the max resistence?
 * changed resistor to 5.9 (it's what I had). This increased resolution.
 * 20ma * 5.9 ohm = 118mv  am i losing 50mv resolution?
 *
 * Calibration:
 *    Connected amp meter between shunt resistor and ground; volt meter across shunt resistor
 *    Connected a 4-20ma generator to simulate current loop
 *    PowerSup+ to 4-20Gen to shunt+ from shunt- to amp meter+ then amp meter- to ground
 *    Board power 3.3v
 *    tweeked _am_shunt_value until readout matched amp meter
 *    original value 5.93ohm tweeked value 5555
 *    reolution now ~ .01ma +/-.005ma
 *    Setup serial input to change map values while testing in pool to calibrate mapf()
 *
 * Current to voltage converter
 * jumpers set to 3.3v
 * can't get good enough range. low end too noisy
 * 4ma=.034v  20ma=2.9v


******************************************/

#include <Arduino.h>
#include <Wire.h>
/**********************  ina3221  ***********************************/

//#include <Beastdevices_INA3221.h>
// Set I2C address to 0x41 (A0 pin -> VCC)
// Beastdevices_INA3221 ina3221(INA3221_ADDR40_GND);

// SDL
#include "SDL_Arduino_INA3221.h"
static const uint8_t _am_addr = 64; //  hex 40 I2C address of sdl board

// tweeked the resistor value while measuring the current with a meter. To make the numbers very close.
static const int32_t _am_shunt_value = 5640; // with throw in sensor  // 5555 = with current gen;

SDL_Arduino_INA3221 ina3221(_am_addr, _am_shunt_value);
// SDL_Arduino_INA3221 ina3221;
// values for 3 chan
float current_ma[3];
float voltage[3];
float shunt[3];
float LoadV[3];
/***********************************   ina219   ***************************/
//#include <Adafruit_INA219.h>
// Adafruit_INA219 ina219;

/******************************  pulsed input  ****************************/
// connected to impeller on flow sensor
#define FLOWSENSORPIN 13

// pulse hi duration
volatile unsigned long pulseInHiTimeBegin = micros();
volatile unsigned long pulseInHiTimeEnd = micros();
unsigned long pulseHiDuration = 0;
volatile bool newPulseHiDurationAvailable = false;

// pulse low duration
volatile unsigned long pulseInLowTimeBegin = micros();
volatile unsigned long pulseInLowTimeEnd = micros();
unsigned long pulseLowDuration = 0;
volatile bool newPulseLowDurationAvailable = false;

// total duration
double TotalDuration = 0;
double TotalDurationOld = 0;
double Frequency = 0;
int DelayCount = 0;

// isr stored in ram
void IRAM_ATTR PulseInterrupt()
{
  // test for hi or low
  if (digitalRead(FLOWSENSORPIN) == HIGH)
  {
    // record time for hi start and low end
    pulseInHiTimeBegin = micros();
    pulseInLowTimeEnd = micros();
    newPulseLowDurationAvailable = true;
  }
  else
  {
    // record time for low start and hi end
    pulseInHiTimeEnd = micros();
    pulseInLowTimeBegin = micros();
    newPulseHiDurationAvailable = true;
  }
}

/**********************************  misc  *************************/
double x;
double in_min = 4.064;
double in_max = 4.504;
double out_min = 0.0;
double out_max = 8.27;

char data_in;
float Distance[3];

double mapf(double var, double InMin, double InMax, double OutMin, double OutMax);

void DisplayData(void);

// app vars
unsigned long update_SD_interval = 6000; // time interval in ms for updating panel indicators
unsigned long update_DISP_interval = 250;
unsigned long update_Sensor_interval = 200;
unsigned long last_Sensor_time = 0;
unsigned long last_DISP_time = 0;
unsigned long last_SD_time = 0;

/*********************************   setup   **************************/
void setup()
{
  Serial.begin(38400);  // on board serial
  Serial1.begin(38400); // BT serial

  /*************************  ina3221  ************************/
  // setup ina3221 SDL lib
  ina3221.begin();

  // bstdev lib
  // ina3221.begin();
  // ina3221.reset();
  //  Set shunt resistors to 100 mOhm for all channels
  // ina3221.setShuntRes(100, 100, 100);

  /********************** flow  **********************/
  // setup pin and interrupt for pulse
  /*   Serial1.print("Flow sensor test!");
    pinMode(FLOWSENSORPIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(FLOWSENSORPIN), PulseInterrupt, CHANGE); */

  /***********************************  ina219  ****************************/
  /*   if (!ina219.begin())
    {
      Serial.println("Failed to find INA219 chip");
      while (1)
      {
        delay(10);
      }
    }
    // To use a slightly lower 32V, 1A range (higher precision on amps):
    // ina219.setCalibration_32V_1A();
    // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
    ina219.setCalibration_16V_400mA();

    Serial.println("Measuring voltage and current with INA219 ..."); */
}

void loop()

{

  /**************************  ina3221   ******************************/

  // SDL lib
  current_ma[0] = ina3221.getCurrent_mA(1) * 1000;
  voltage[0] = ina3221.getBusVoltage_V(1);
  shunt[0] = ina3221.getShuntVoltage_mV(1);
  LoadV[0] = voltage[0] + (shunt[0] / 1000000);

  // bstdev lib
  /*     current[2] = ina3221.getCurrent(INA3221_CH3);
      voltage[2] = ina3221.getVoltage(INA3221_CH3);
      shunt[2] = ina3221.getShuntVoltage(INA3221_CH3);
      LoadV[2] = voltage[2] + (shunt[2] / 1000000);
  */

  /***************************  ina219  *******************************/

  /*   shunt[2] = ina219.getShuntVoltage_mV();
    voltage[2] = ina219.getBusVoltage_V();
    current_ma[2] = ina219.getCurrent_mA();
  */

  /*   if (current_ma[2] < 20.0 && current_mA > 4.0)
    {
      DistanceI = mapf(current_ma[2], 4.0, 20.0, 49, 301);
    }
    else
    {
      DistanceI = 0;
    }

    if (voltage[2] < 3.0 && voltage[2] > 0.5)
    {
      DistanceV = mapf(voltage[2], .5, 3.0, 49, 301);
    }
    else
    {
      DistanceV = 0;
    }

    power_mW = ina219.getPower_mW();
    loadvoltage = voltage[2] + (shunt[2] / 1000);
  */

  while (Serial1.available() > 0)
  {
    //  runner.pause();
    data_in = Serial1.read(); // Get next character

    // pump on level
    if (data_in == 'N')
    {                                //  Slider
      in_min = Serial1.parseFloat(); // parseInt();
    }

    // pump off level
    if (data_in == 'O')
    { //  Slider
      in_max = Serial1.parseFloat();
    }

    // alarm on level
    if (data_in == 'P')
    { //  Slider
      out_min = Serial1.parseFloat();
    }

    // alarm off level
    if (data_in == 'Q')
    { //  Slider
      out_max = Serial1.parseFloat();
    }
  }
  // readings seem more stable at cm (300) than at mm (3000)
  Distance[0] = mapf(current_ma[0], in_min, in_max, out_min, out_max);

  unsigned long ap = millis();
  if ((ap - last_DISP_time) > update_DISP_interval)
  {
    last_DISP_time = ap;

    DisplayData();
  }

  /////////////////////////////// pulse flow sensor
  // have a hi time
  /*   if (newPulseHiDurationAvailable)
    {
      newPulseHiDurationAvailable = false;
      pulseHiDuration = pulseInHiTimeEnd - pulseInHiTimeBegin;
    }
    // have a low time
    if (newPulseLowDurationAvailable)
    {
      newPulseLowDurationAvailable = false;
      pulseLowDuration = pulseInLowTimeEnd - pulseInLowTimeBegin;
    }

    // total time
    TotalDuration = pulseHiDuration + pulseLowDuration;

    // test if same reading - means not moving
    if ((TotalDuration - TotalDurationOld) == 0)
    {
      DelayCount++;
      // helps remove on/off chatter when numbers close
      if (DelayCount > 64000)
      {
        // Serial1.print("DelayCount: ");
        // Serial1.println(DelayCount);

        // clear vals
        DelayCount = 0;
        TotalDuration = 0;
        Frequency = 0;
        pulseHiDuration = 0;
        pulseLowDuration = 0;
      }
    }
    else
    {
      // don't divide by 0
      if (TotalDuration != 0)
      {
        Frequency = 1 / TotalDuration;
        // convert from micro seconds
        Frequency = Frequency * 1000000;

        // Serial1.print(" TotalDuration ");
        // Serial1.print(TotalDuration);
        // Serial1.print(" FreqON:");
        // Serial1.println(Frequency);
        // Serial1.print(" LPM: ");
        // Serial1.println(Frequency / 5.5);
      }

      TotalDurationOld = TotalDuration;
      DelayCount = 0;
    }
  */
}

/*********************************   subs   *************************************/

void DisplayData(void)
{
  // local
  Serial.print("C1: ");
  Serial.print(current_ma[0], 3);
  Serial.print("mA, ");
  Serial.print(voltage[0], 2);
  Serial.print("V, ");
  Serial.print("Sh ");
  Serial.print(shunt[0], 2);
  Serial.print("mV, ");
  Serial.print("LodV ");
  Serial.print(LoadV[0], 2);
  Serial.print("V, ");
  Serial.print("D: ");
  Serial.println(Distance[0]);
  Serial.println("");
  Serial.println("");
  Serial.print("IM ");
  Serial.print(in_min);
  Serial.print(" IX ");
  Serial.print(in_max);
  Serial.print(" OM ");
  Serial.print(out_min);
  Serial.print(" OX ");
  Serial.print(out_max);
  Serial.println("");
  Serial.println("");

  // BT
  //  Serial.print("C1: ");
  Serial1.print(current_ma[0], 3);
  Serial1.print("mA, ");
  Serial1.print(voltage[0], 2);
  Serial1.print("V, ");
  Serial1.print("Sh ");
  Serial1.print(shunt[0], 2);
  Serial1.print("mV, ");
  // Serial1.print("LV ");
  // Serial1.print(LoadV[0], 2);
  // Serial1.println("V");
  Serial1.print("D ");
  Serial1.println(Distance[0]);
  Serial1.println("");
  Serial1.println("");
  Serial1.print("IM ");
  Serial1.print(in_min);
  Serial1.print(" IX ");
  Serial1.print(in_max);
  Serial1.print(" OM ");
  Serial1.print(out_min);
  Serial1.print(" OX ");
  Serial1.print(out_max);
  Serial1.println("");
  Serial1.println("");
  /*
       Serial.print(" C2: ");
      Serial.print(current_ma[1], 4);
      Serial.print("ma, ");
      Serial.print(voltage[1], 2);
      Serial.print("V, ");
      Serial.print("Sh ");
      Serial.print(shunt[1], 2);
      Serial.print("uV, ");
      Serial.print("LV ");
      Serial.print(LoadV[1], 2);
      Serial.println("V"); */
  /*
      Serial1.print("  C3: ");
      Serial1.print(current[2], 4);
      Serial1.print("A, ");
      Serial1.print(voltage[2], 2);
      Serial1.print("V");
      Serial1.print(" Shnt: ");
      Serial1.print(shunt[2], 2);
      Serial1.print("uV, ");
      Serial1.print("LodV ");
      Serial1.print(LoadV[2], 2);
      Serial1.println("V");
     */
}

double mapf(double var, double InMin, double InMax, double OutMin, double OutMax)
{
  return (var - InMin) * (OutMax - OutMin) / (InMax - InMin) + OutMin;
}