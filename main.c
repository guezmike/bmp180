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
#define Pressure_Control_Word 0x34
#define Read_MSB_Reg 0xF6
#define Read_LSB_Reg 0xF7

long B5;

struct bmp180_calib
{
    short   AC1;
    short   AC2;
    short   AC3;
    unsigned short AC4;
    unsigned short AC5;
    unsigned short AC6;
    short   B1;
    short   B2;
    short   MB;
    short   MC;
    short   MD;
    short   OSS;
}   bmp180_calib_params = {0};


void bmp180_get_cal_param(){
	uint8_t temp_lsb = 0;
	uint8_t temp_msb = 0;
	i2c_read(BMP_Read_Addr,0xAA,&temp_msb,0x01); 		   
	i2c_read(BMP_Read_Addr,0xAB,&temp_lsb,0x01);
	bmp180_calib_params.AC1 = (temp_msb<<8|temp_lsb);
	i2c_read(BMP_Read_Addr,0xAC,&temp_msb,0x01); 		   
	i2c_read(BMP_Read_Addr,0xAD,&temp_lsb,0x01);
	bmp180_calib_params.AC2 = (temp_msb<<8|temp_lsb);
	i2c_read(BMP_Read_Addr,0xAE,&temp_msb,0x01); 		   
	i2c_read(BMP_Read_Addr,0xAF,&temp_lsb,0x01);
	bmp180_calib_params.AC3 = (temp_msb<<8|temp_lsb);
	i2c_read(BMP_Read_Addr,0xB0,&temp_msb,0x01); 		   
	i2c_read(BMP_Read_Addr,0xB1,&temp_lsb,0x01);
	bmp180_calib_params.AC4 = (temp_msb<<8|temp_lsb);
	i2c_read(BMP_Read_Addr,0xB2,&temp_msb,0x01); 		   
	i2c_read(BMP_Read_Addr,0xB3,&temp_lsb,0x01);
	bmp180_calib_params.AC5 = (temp_msb<<8|temp_lsb);
	i2c_read(BMP_Read_Addr,0xB4,&temp_msb,0x01); 		   
	i2c_read(BMP_Read_Addr,0xB5,&temp_lsb,0x01);
	bmp180_calib_params.AC6 = (temp_msb<<8|temp_lsb);
	
	i2c_read(BMP_Read_Addr,0xB6,&temp_msb,0x01); 		   
	i2c_read(BMP_Read_Addr,0xB7,&temp_lsb,0x01);
	bmp180_calib_params.B1 = (temp_msb<<8|temp_lsb);
	i2c_read(BMP_Read_Addr,0xB6,&temp_msb,0x01); 		   
	i2c_read(BMP_Read_Addr,0xB7,&temp_lsb,0x01);
	bmp180_calib_params.B2 = (temp_msb<<8|temp_lsb);
	i2c_read(BMP_Read_Addr,0xBA,&temp_msb,0x01); 		   
	i2c_read(BMP_Read_Addr,0xBB,&temp_lsb,0x01);
	
	bmp180_calib_params.MB = (temp_msb<<8|temp_lsb);
	i2c_read(BMP_Read_Addr,0xBC,&temp_msb,0x01); 		   
	i2c_read(BMP_Read_Addr,0xBD,&temp_lsb,0x01);
	bmp180_calib_params.MC = (temp_msb<<8|temp_lsb);
	i2c_read(BMP_Read_Addr,0xBE,&temp_msb,0x01); 		   
	i2c_read(BMP_Read_Addr,0xBF,&temp_lsb,0x01);
	bmp180_calib_params.MD = (temp_msb<<8|temp_lsb);

	i2c_read(BMP_Read_Addr,0xF4,&temp_lsb,0x01);
	bmp180_calib_params.OSS = temp_lsb;


}



long bmp180_get_ut(){
	long temperature_data = 0;
	uint8_t data_msb = 0;
	uint8_t data_lsb = 0;
	i2c_write(BMP_Write_Addr,BMP_Control_Reg,Temp_Control_Word,0x01);  //write 2e into reg 0xf4
	delay_us(4500);  							   //wait 4.5ms
	i2c_read(BMP_Read_Addr,Read_MSB_Reg,&data_msb,0x01); 		   //read 0xF6
	i2c_read(BMP_Read_Addr,Read_LSB_Reg,&data_lsb,0x01);                //read oxF7
	temperature_data = (data_msb<<8 | data_lsb);
	return temperature_data;
}


long bmp180_get_up(){
	long pressure_data = 0;
	uint8_t data_msb = 0;
	uint8_t data_lsb = 0;
	uint8_t data_xlsb = 0;
	i2c_write(BMP_Write_Addr,BMP_Control_Reg,(Pressure_Control_Word+(bmp180_calib_params.OSS<<6)),0x01);  //write 2e into reg 0xf4
	delay_us(4500);  							   //wait 4.5ms
	i2c_read(BMP_Read_Addr,Read_MSB_Reg,&data_msb,0x01); 		   //read 0xF6
	i2c_read(BMP_Read_Addr,Read_LSB_Reg,&data_lsb,0x01);                //read oxF7
	i2c_read(BMP_Read_Addr,0xF8,&data_xlsb,0x01);                       //read oxF8
	pressure_data = ((data_msb<<16 | data_lsb<<8 | data_xlsb)) >>(8-bmp180_calib_params.OSS);
	return pressure_data;	
}


long bmp180_get_temperature(long UT){
	long x1,x2;
	x1 = ((UT - bmp180_calib_params.AC6)*bmp180_calib_params.AC5)>>15;
	x2 = ((long)bmp180_calib_params.MC<<11)/(x1 + bmp180_calib_params.MD);
	b5 = x1+x2;
	return (b5+8)>>4;
}

long bmp180_calpressure(long up){
	long x1,x2,x3,b3,b6,p;
	unsigned long b4,b7;
	b6 = b5 - 4000;
	x1 = (bmp180_calib_params.B2*((bmp180_calib_params.B6*bmp180_calib_params.B6)>>12))>>11;
	x2 = (bmp180_calib_params.AC2*b6)>>11;
	x3 = x1+x2;
	b3 = ((((long)bmp180_calib_params.AC1*4+x3)<<bmp180_calib_params.OSS)+2)>>2;
	x1 = (bmp180_calib_params.AC3*b6)>>13;
	x2 = (bmp180_calib_params.B1*((bmp180_calib_params.B6*bmp180_calib_params.B6)>>12)>>16;
	x3 = ((x1+x2)+2)>>2;
	b4 = (bmp180_calib_params.AC4*((unsigned long)(x3+32768)))>>15;
	b7 = ((unsigned long)up-b3)*(50000>>bmp180_calib_params.OSS);
	if(b7 < 0x80000000){
		p = (b7<<1) / b4;
	}
	else{
		p = (b7/b4)<<1;
	}
	x1 = (p>>8)*(p>>8);
	x1 = (x1*3038)>>16;
	x2 = (-7357*p)>>16;
	p = p + (x1+x2+3791>>4);
	return p;
}




int sensor_available(){
	uint8_t id = 0;
	i2c_read(BMP_Read_Addr,Read_LSB_Reg,&id,0x01);
	if(id==0x55){
		return 1;
	} 
	else{
		return 0;
	}

}



void main(){
	long ut,up,temperature_celcius,pressure_pascals,pressure_heptopascal;
	while(1){
		if(sensor_available()){
			ut = bmp180_get_ut();
			up = bmp180_get_up();
			temperature_celcius = bmp180_get_temperature(ut)/10;
			pressure_pascals = bmp_calcpressure(up);
			pressure_heptopascal = pressure_pascals/100;
			delay_ms(1000);
		}
	}
}


