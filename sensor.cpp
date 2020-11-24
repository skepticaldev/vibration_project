#include <stdio.h>
#include <sys/types.h>
#include <chrono>
#include <iostream>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

/* MPU6050 Registers and addresses */
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

// MPU6050 device address
#define DEVICE_ADDRESS 0x68


//--------------------------
// Function declarations
//--------------------------

//MPU config function
void mpu_init(int file);

// Write config values to registers
void write_byte_data(int file, __u8 reg, __u8 byte);

// Read MPU raw values LSB/g
int16_t read_raw_data(int file, __u8 reg);

// Get signed values
int16_t signed_value(int16_t value);

using namespace std;


// Main function
int main() {
	int file_i2c;
	int length;
	unsigned char buffer[60] ={0};

	//------ OPEN THE I2C BUS ------
	char *filename = (char*)"/dev/i2c-1";
	if((file_i2c = open(filename, O_RDWR)) < 0) {
		//ERROR HANDLING
		printf("Failed to open the i2c bus");
		return 1;
	}

	if (ioctl(file_i2c, I2C_SLAVE, DEVICE_ADDRESS) < 0) {
		printf("Failed to acquire bus access and/or talk to slave.\n");
		return 2;
	}

	// MPU configuration
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

void mpu_init(int file) {
	// write to sample rate register
	write_byte_data(file, SMPLRT_DIV, 0x00);

	//write to power management register
	write_byte_data(file, PWR_MGMT_1, 0x01);
	
	//write to configuration register
	write_byte_data(file, CONFIG, 0x00);

	//write to gyro configuration register
	write_byte_data(file, GYRO_CONFIG, 0x18);

	//write to accel configuration register
	write_byte_data(file, ACCEL_CONFIG, 0x18);

	//write to interrupt enable register
	write_byte_data(file, INT_ENABLE, 0x01);
	
	//write to DLPF register
	write_byte_data(file, DLPF_CFG, 0x00);

	//write to FIFO_EN
	write_byte_data(file, FIFO_EN, 0x08);
}


void write_byte_data(int file, __u8 reg, __u8 byte){
	//payload to deliver to register
	char payload[2];
	
	payload[0] = reg;
	payload[1] = byte;
	
	if(write(file, payload, 2) != 2) {
		printf("Error to write register");
	}
}

int16_t read_raw_data(int file, __u8 reg){
	// buffer to receive high and low values
	int8_t buffer[2];
	
	char payload[1];
	
	payload[0] = reg;

	write(file, payload, 1);
	read(file,buffer,2);

	int16_t value = (buffer[0]<<8)+buffer[1];

	return value;
}

int16_t signed_value(int16_t value){
	if(value>32768){
		value = value - 65536;
	}
	return value;
}

double avg_data(){
	double avg_ax = 0, avg_ay = 0, avg_az = 0;

	int i = 0;
}