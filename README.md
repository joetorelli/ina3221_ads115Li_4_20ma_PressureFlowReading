# ina3221_ads115Li_4_20ma_PressureFlowReading
Testing several sensors reading 4-20ma with ina219/3221 also ads115
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
 * 20ma * 5.9 ohm = 118mv  am i losing 50mv headroom?
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
