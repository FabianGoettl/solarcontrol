# Arduino solar control
This is a solar control loop, which uses solar and puffer temperature inputs as readings to control one pump relay output. We read analog KTY10 sensors by measuring the voltage at the ADC. Finally, we determine the resistance and temperature based on the polynomial in the [datasheet][1].

[1]: http://pdf.datasheetcatalog.com/datasheet/infineon/1-kt.pdf.
  

## Requirements
- Arduino
- Pin 0: KTY10 (or similiar) sensor for 
- Pin 1: KTY10 (or similiar) sensor 
- Pin 2: Relay for water pump
- 2 Lines I2C Liquid Crystal Display
