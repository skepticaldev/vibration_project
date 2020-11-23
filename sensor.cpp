#include <stdio.h>
#include <sys/types.h>
#include <chrono>
#include <iostream>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define PWR_MGMT_1 0x6B
#define SMPLRT_DIV 0x19
#define DLPF_CFG 0x26
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define INT_ENABLE 0x38
#define FIFO_EN 0x23
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H 0x43
#define GYRO_YOUT_H 0x45
#define GYRO_ZOUT_H 0x47

#define DEVICE_ADDRESS 0x68

void write_byte_data(int file, __u8 reg, __u8 byte)
{
	char buf[2];
	
	buf[0] = reg;
	buf[1] = byte;
	
	if(write(file, buf, 2) != 2)
	{
		printf("Error to write register");
	}
}
 
void read_raw_data(int file, __u8 reg)
{
	int8_t buff[2];
	
	char r[1];
	r[0] = reg;

	write(file, r, 1);
	read(file,buff,2);

	int value = (buff[0]<<8)+buff[1];
}

void mpu_init(int file)
{
	write_byte_data(file, SMPLRT_DIV, 0x00);
	write_byte_data(file, PWR_MGMT_1, 0x01);
	write_byte_data(file, CONFIG, 0x00);
	write_byte_data(file, GYRO_CONFIG, 0x18);
	write_byte_data(file, ACCEL_CONFIG, 0x18);
	write_byte_data(file, INT_ENABLE, 0x01);
	write_byte_data(file, DLPF_CFG, 0x00);
	write_byte_data(file, FIFO_EN, 0x08);
}

using namespace std;

int main()
{
	int file_i2c;
	int length;
	unsigned char buffer[60] ={0};

	//------OPEN THE I2C BUS------
	char *filename = (char*)"/dev/i2c-1";
	if((file_i2c = open(filename, O_RDWR)) < 0)
	{
		//ERROR HANDLING
		printf("Failed to open the i2c bus");
		return 1;
	}

	if (ioctl(file_i2c, I2C_SLAVE, DEVICE_ADDRESS) < 0)
	{
		printf("Failed to acquire bus access and/or talk to slave.\n");
		return 2;
	}

	mpu_init(file_i2c);
	auto start = chrono::steady_clock::now();
	int i = 0;
	for(i=0;i<=999;i++){
		read_raw_data(file_i2c, ACCEL_ZOUT_H);
		read_raw_data(file_i2c, ACCEL_XOUT_H);
		read_raw_data(file_i2c, ACCEL_YOUT_H);
	}
	auto end = chrono::steady_clock::now();
	cout<<"Elapsed milliseconds: "
		<<chrono::duration_cast<chrono::milliseconds>(end-start).count()
		<<" ms"<<endl;
	printf("Hello World!\n");
	return 0;
}


