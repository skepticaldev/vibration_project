#include <stdio.h>
#include <sys/types.h>
#include <time.h>
#include <chrono>
#include <iostream>
#include <tuple>

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

//Calibration parameter
#define RANGE_ERROR 10

using namespace std;

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

// Calculate average sample
tuple <double,double,double> avg_data(int buffer_size, int file, double X_OFFSET, double Y_OFFSET, double Z_OFFSET);

// Calibrate function
tuple <double, double, double> calibrate(int file, int range_error, int buffer_size);

//Collect Data
void collect_samples(
	int file, 
	int size, 
	int16_t sample_buffer[][3],
	chrono::steady_clock::time_point time_buffer[], double control_time);

// Main function
int main() {
	int file_i2c;
	int length;

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

	int16_t sample_buffer[1024][3];
	chrono::steady_clock::time_point time_buffer[1024];

	chrono::steady_clock::time_point start = chrono::steady_clock::now();
	collect_samples(file_i2c, 1024, sample_buffer, time_buffer, 0.0);
	chrono::steady_clock::time_point end = chrono::steady_clock::now();

	for(int i=0; i<1024;i++){
		cout<<sample_buffer[i][0]<<" "<<sample_buffer[i][1]<<" "<<sample_buffer[i][2]<<" "
		<< chrono::duration_cast<chrono::milliseconds>(time_buffer[i]-start).count()<<endl;
	}

	cout<<"Elapsed millliseconds: " << chrono::duration_cast<chrono::milliseconds>(time_buffer[1023]-start).count()<< " ms"<<endl;

//	for(int i = 0; i<1000;i++){
//		cout<<samples[i][0]<< " " << samples[i][1] << " " << samples[i][2] << endl;
//	}

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


void collect_samples(
	int file, 
	int size, 
	int16_t sample_buffer[][3],
	chrono::steady_clock::time_point time_buffer[],
	double control_time){
	int i=0;

	struct timespec req = {0};
	req.tv_sec = 0;
	req.tv_nsec = control_time * 1000000L;

	while (i<size) {
		sample_buffer[i][0] = read_raw_data(file, ACCEL_XOUT_H);
		sample_buffer[i][1] = read_raw_data(file, ACCEL_YOUT_H);
		sample_buffer[i][2] = read_raw_data(file, ACCEL_ZOUT_H);
		time_buffer[i] = chrono::steady_clock::now();
		nanosleep(&req, (struct timespec *)NULL);
		i+=1;
	}
}

int16_t signed_value(int16_t value){
	if(value>32768){
		value = value - 65536;
	}
	return value;
}

tuple <double,double,double> avg_data(int buffer_size, int file, double X_OFFSET=0, double Y_OFFSET=0, double Z_OFFSET=0){
	
	double avg_ax = 0, avg_ay = 0, avg_az = 0;
	
	double buff_ax = 0, buff_ay = 0, buff_az = 0;
	
	int i = 0;

	while (i<buffer_size+101) {
		
		if(i>100 && i<=(buffer_size+100)) {
			buff_ax = buff_ax + signed_value(read_raw_data(file, ACCEL_XOUT_H)) + X_OFFSET;
			buff_ay = buff_ay + signed_value(read_raw_data(file, ACCEL_YOUT_H)) + Y_OFFSET;
			buff_az = buff_az + signed_value(read_raw_data(file, ACCEL_ZOUT_H)) + Z_OFFSET;
		}

		if (i==(buffer_size+100)) {
			avg_ax = buff_ax/buffer_size;
			avg_ay = buff_ay/buffer_size;
			avg_az = buff_az/buffer_size;
		}

		i+=1;

		sleep(0.002);
	}

	return make_tuple(avg_ax, avg_ay, avg_az);
}

tuple <double, double, double> calibrate(int file, int range_error, int buffer_size){

	double avg_x, avg_y,avg_z;
	double x_offset = 0, y_offset = 0, z_offset = 0;

	tie(avg_x, avg_y, avg_z) = avg_data(1000, file, 0, 0, 0);

	x_offset = -avg_x/range_error;
	y_offset = -avg_y/range_error;
	z_offset = (2048.0 - avg_z)/range_error;

	while (true) {
		int ready = 0;

		tie(avg_x, avg_y, avg_z) = avg_data(buffer_size, file, x_offset, y_offset, z_offset);

		if(abs(avg_x)<=range_error){
			ready+=1;
		} else {
			x_offset = x_offset - avg_x/range_error;
		}
		
		if(abs(avg_y)<=range_error){
			ready+=1;
		} else {
			y_offset = y_offset - avg_y/range_error;
		}

		if(abs(avg_z)<=range_error){
			ready+=1;
		} else {
			z_offset = z_offset - avg_z/range_error;
		}

		if(ready==3){
			return make_tuple(x_offset, y_offset, z_offset);
		}
	}
}
