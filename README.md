Chemical sensor will have digital and signal detection circuitry. The sensor housing will be made water-tight. Factory calibration of the detector circuit is done, but the sensors  are replaced from time to time so there will be a need for sensor calibration against a known Chemical value (or two). Calibration is made similar to that of the oxygen or pH sensors with the user being prompted to place the sensor in the calibration solution then have the unit record a value. The sensor response to chemical is linear so the calibration  only require corrections for slope and offset.

Power: The device is battery powered and include a magnetic Reed switch (https://www.inventables.com/technologies/magnetic-switch-normally-open ) to engage battery power. The battery is rechargeable and since the analog circuitry is not expected to have too high a demand, multiple coin cells might do the job. Lithium battery should not be used, which leaves Nickel-Zinc as the most likely candidate (as of now). 

Non-volatile Memory: Assuming one reading every 10 seconds, 40 bytes/measurement (including time, date, chemical, temperature etc.)/10s, we expect (8*60*60/10)*40 = 115.2kbytes of non-volatile storage memory (or may be 256kbytes to provide some margin).

Analog inputs: There is one Calcium channel and one temperature channel. The dynamic range is determined. The sensor detection  uses a phase lock loop with a VCO feedback. The voltage value reported would be tailored to match the A-to-D of the CPU or a separate A-to-D may be used, if higher resolution is needed. The temperature sensor  includes a simple Wheatstone bridge the output of which is tailored to match the A-to-D.

Real Time Clock: A real time clock should be included to time stamp the calcium and temperature data.

User Interface: Use of an RS232 interface and correspondingly development of GUI are anticipated as well.

