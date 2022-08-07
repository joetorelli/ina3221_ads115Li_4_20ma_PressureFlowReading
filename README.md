/**************************************************
* Testing:
 *  water flow - pulse flow sensor,  level - ThrowIn Senor
 *  ina3221 3chan analog monitor I2C
 *  ads1115 4chan 16bit adc I2C
 *  hw-685 Current To Volts Converter
 *
 * Reading sensors
 *  gr-108 paddle wheel plastic 1-60lpm 1.2Mpa f=5.5*q q=lpm
 *  cf-b10 impeller brass 2-50lpm 1.75Mpa f=7.5*q-4
 *  g1/4 Pressure Transducer(x2) 80psi 0-5v and 145psi 4-20ma
 *  1/4 pressure xdcr 200psi 4-20ma 12v
 *
 * Interfaces
 *  Current to voltage converter 0/4-20ma to 0-3.3/5/10v connected esp32 adc
 *  ina3221 voltage/current monitor connected to 80psi 5v and 145psi 4-20ma and ThrowIn 4-20ma
 *  also used Utronics Current Signal gen in place of sensors.
 *  feather esp32 for arduino
 *
 * //Explanation of I2C address for INA3221:
 *   //INA3221_ADDR40_GND = 0b1000000, // A0 pin -> GND
 *   //INA3221_ADDR41_VCC = 0b1000001, // A0 pin -> VCC
 *   //INA3221_ADDR42_SDA = 0b1000010, // A0 pin -> SDA
 *   //INA3221_ADDR43_SCL = 0b1000011  // A0 pin -> SCL
 *
 *********************************************
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
 *
 * SDL_Arduino_INA3221.h
 * dosen't seem to have the resolution
 * using CurrentGen or ThrowIn jumps in .4ma steps
 * may need to change shunt resistor to:
 * If I'm reading datasheet the max volts is 168mv across the shunt
 *  shut max volts / max current = resistor
 *           168mv / 20ma        = 8.4 ohm   is this the max resistence?
 * changed resistor to 5.9 (it's what I had). This increased resolution.
 * 20ma * 5.9 ohm = 118mv  am i losing 50mv headroom?
 * 20ma * 6.19 ohm = 123mv
 * 20ma * 7.5 ohm = 150mv
 *
 * Calibration:
 *    Connected amp meter between shunt resistor and ground; volt meter across shunt resistor
 *    Connected a 4-20ma generator to simulate current loop
 *    PowerSup+ to 4-20Gen to shunt+ from shunt- to amp meter+ then amp meter- to ground
 *    Board power 3.3v
 *    tweeked _am_shunt_value until readout matched amp meter
 *    original value 5.93ohm tweeked value 5640 = with throw in sensor  // 5555 = with current gen;
 *    reolution now ~ .01ma +/-.005ma
 *    Setup serial input to change map values while testing in pool to calibrate mapf()
 *
 *  * SDL Lib works good with changes. I can't see how to change the shunt values seperately.
 *    using mapf() and scaling ma reading to 300cm gives very good stable results
 *    instead of using 4ma at the bottom @ 4.1ma I measured the distance to the end of sensor
 *    and used the 2 vals as my mins. The max vals were from the deepset water ~42in I had and the ma reading.
 *
 * *********************************
 * Current to voltage converter
 * jumpers set to 3.3v
 * can't get good enough range. low end too noisy 4ma=.034v  20ma=2.9v
 * need to try different jumpers to bring up the low end.
 *
 *
 * *********************************************************/
