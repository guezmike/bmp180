//API for BMP180 Pressure Sensor

/* Task:

// I2C burst read (reads from 7 bit I2C device 'address' 'size' bytes starting with register 'reg' into data)
	i2c_read(uint8_t address, uint8_t reg, uint8_t *data, size_t size);
// I2C burst write (writes to 7 but I2C device 'address' 'size' bytes, starting with register 'reg')
	i2c_write(uint8_t address, uint8_t reg, uint8_t *data, size_t size);

	
	Write two interface functions that allow to:
	(1) read temperature in degrees celsius (C) and 
	(2) pressure in hecto-pascal (hPa) as a double.
	(3) Ensure the BMP180 is actually available on the I2C bus.
	(4)Read measured data as burst reads and convert to temperature and pressure.
	(5)Code should be a single GIST (https://gist.github.com/) that is compilable into 
	an object file using the prototypes above and (preferably) the arm-none-eabi-gcc. 
	Please provide your solution using your github account.
*/


#include i2c_driver.h

//BMP180 module address: 0xEF(read) oxEE(write)
//Control Register(0xF4)-> temp-0x2e, pressure-0x34

#define BMP_Read_Addr 0xEE
#define BMP_Write_Addr 0xEF

#define BMP_Control_Reg 0xF4
#define Temp_Control_Word 0x2E
#define Read_MSB_Reg 0xF6
#define Read_LSB_Reg 0xF7



long read_temp(){
	long temp_data = 0;
	uint8_t data_msb = 0;
	uint8_t data_lsb = 0;
	i2c_write(BMP_Write_Addr,BMP_Control_Reg,Temp_Control_Word,0x01); 
	wait(4.5ms);
	i2c_read(BMP_Read_Addr,Read_MSB_Reg,data_msb,0x01);
	i2c_read(BMP_Read_Addr,Read_MSB_Reg,data_lsb,0x01);
	temp_data = (data_msb<<8 | data_lsb);



}



int read_pressure(){




}


void check_sensor(){


}



void main(){



}
