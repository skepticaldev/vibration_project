#include <stdio.h>
#include <sys/types.h>
#include <time.h>
#include <chrono>
#include <iostream>
#include <tuple>
#include <fstream>
#include <string>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

//Power 
#define POWER_CLT 0x2D

//Output data rate
#define BW_RATE 0x2C 

//Data format
#define DATA_FORMAT 0x31

//Data registry
#define ACCEL_XOUT_L 0x32
#define ACCEL_YOUT_L 0x34
#define ACCEL_ZOUT_L 0x36

// ADXL345 device address
#define DEVICE_ADDRESS 0x53

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
	chrono::steady_clock::time_point time_buffer[], 
	//millisenconds
	double control_time);

// Calculate time between samples to reach desired frequency
double calculate_control_time(int file, 
	int buffer_size,
	// Frequency in hertz 
	int target_frequency,
	// allowed error in hertz 
	int range_error);

//Export data to csv file
void export_csv_data(double data_buffer[][4], int buffer_size, int filename);

//Normalize samples
void normalize_samples(
	double data_buffer[][4], 
	int buffer_size, 
	int16_t sample_buffer[][3], 
	chrono::steady_clock::time_point time_buffer[], 
	double x_offset, 
	double y_offset, 
	double z_offset, 
	int scale_factor);

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
	cout<<"Initializing..."<<endl;
	mpu_init(file_i2c);

	int buffer_size = 1024;

	double x_offset = 0, y_offset = 0, z_offset = 0;

	//Calibration
	//cout<<"Calibrating..."<<endl;
	//tie(x_offset, y_offset, z_offset) = calibrate(file_i2c, RANGE_ERROR, 2000);
	//cout<<"Offsets xyz: " << x_offset<<" "<<y_offset<<" "<<z_offset<<endl;

	//Calculate time to control frequency
	//cout<<"Calculating control time..."<<endl;
	//double c_time = calculate_control_time(file_i2c, 500, 1000, 5);
	//cout<<"Control time:"<< c_time << endl;

	char key;
	cout<<"Press a key to continue or s to stop: ";
	cin >> key;

	int file_number = 1;

	while(key!='s'){

		// Buffer to collect samples
		int16_t sample_buffer[buffer_size][3];

		// Buffer to collect aquicsition times
		chrono::steady_clock::time_point time_buffer[buffer_size];

		cout<<"Collecting samples..."<<endl;
		collect_samples(file_i2c, buffer_size, sample_buffer, time_buffer, 0);

		//for(int i=0; i<buffer_size;i++){
		//	cout<<""<< (double)chrono::duration_cast<chrono::microseconds>(time_buffer[i]-time_buffer[0]).count()/1000
		//	<<""<<sample_buffer[i][0]<<" "<<sample_buffer[i][1]<<" "<<sample_buffer[i][2]<<endl;
		//}

		double data_buffer[buffer_size][4];

		cout<<"Normalizing samples..."<<endl;
		normalize_samples(data_buffer, buffer_size, sample_buffer, time_buffer, x_offset, y_offset, z_offset, 256);

		//for(int i=0; i<1024;i++){
		//	cout<<data_buffer[i][0]<<" "<<data_buffer[i][1]<<" "<<data_buffer[i][2]<<" "<<data_buffer[i][3]<<endl;
		//}

		cout<<"Exporting data..."<<endl;
		export_csv_data(data_buffer, buffer_size, file_number);

		cout<<"Press s to stop or enter to continue: ";
		cin >> key;

		file_number+=1;
	}

	return 0;
}

void mpu_init(int file) {

	//power
	write_byte_data(file, POWER_CLT, 0x08);

	//write to Output data rate
	write_byte_data(file, BW_RATE, 0x0f);

	//write to data format
	write_byte_data(file, DATA_FORMAT, 0x0b);
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

	int16_t value = (buffer[1]<<8)+buffer[0];

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

	for (i=0;i<size;i++) {
		sample_buffer[i][0] = read_raw_data(file, ACCEL_XOUT_L);
		sample_buffer[i][1] = read_raw_data(file, ACCEL_YOUT_L);
		sample_buffer[i][2] = read_raw_data(file, ACCEL_ZOUT_L);
		time_buffer[i] = chrono::steady_clock::now();
		//nanosleep(&req, (struct timespec *)NULL);
	}
}

tuple <double,double,double> avg_data(int buffer_size, int file, double X_OFFSET=0, double Y_OFFSET=0, double Z_OFFSET=0){
	
	double avg_ax = 0, avg_ay = 0, avg_az = 0;
	
	double buff_ax = 0, buff_ay = 0, buff_az = 0;
	
	int i = 0;

	while (i<buffer_size+101) {
		
		if(i>100 && i<=(buffer_size+100)) {
			buff_ax = buff_ax + read_raw_data(file, ACCEL_XOUT_L) + X_OFFSET;
			buff_ay = buff_ay + read_raw_data(file, ACCEL_YOUT_L) + Y_OFFSET;
			buff_az = buff_az + read_raw_data(file, ACCEL_ZOUT_L) + Z_OFFSET;
		}

		if (i==(buffer_size+100)) {
			avg_ax = buff_ax/buffer_size;
			avg_ay = buff_ay/buffer_size;
			avg_az = buff_az/buffer_size;
		}

		i+=1;
	}

	return make_tuple(avg_ax, avg_ay, avg_az);
}

tuple <double, double, double> calibrate(int file, int range_error, int buffer_size){

	double avg_x, avg_y,avg_z;
	double x_offset = 0, y_offset = 0, z_offset = 0;

	tie(avg_x, avg_y, avg_z) = avg_data(buffer_size, file, 0, 0, 0);

	x_offset = -avg_x/range_error;
	y_offset = -avg_y/range_error;
	z_offset = -avg_z/range_error;

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

double calculate_control_time(int file, int buffer_size, int target_frequency, int range_error) {

	//Control time in milliseconds
	double c_time = 0.0;

	//target period time in microseconds us
	double target_us_time = 1*1000000/(double)target_frequency;
	
	while(true) {
		// get start time to calculate total time
		chrono::steady_clock::time_point start =  chrono::steady_clock::now();
		
		struct timespec req = {0};
		req.tv_sec = 0;
		req.tv_nsec = c_time * 1000000L;
		
		for(int i=0;i<buffer_size;i++) {
			read_raw_data(file, ACCEL_XOUT_L);
			read_raw_data(file, ACCEL_YOUT_L);
			read_raw_data(file, ACCEL_ZOUT_L);
			nanosleep(&req, (struct timespec *) NULL);
		}

		// get end time to calculate total time
		chrono::steady_clock::time_point end =  chrono::steady_clock::now();
		
		// Calculate total acquisition time
		int64_t total = chrono::duration_cast<chrono::microseconds>(end-start).count();
		
		// acquisition period
		double t_time = total/(double)buffer_size;

		// Calculate acquisition frequency (convert t_time to seconds) 
		double freq = 1*1000000/(double)t_time;
		
		//If target frequency is higher, there is nothing to do
		if((freq<target_frequency) || abs(freq-target_frequency)<=range_error){
			return c_time;
		}

		double error = abs(target_us_time - t_time);

		// add time to control_time to decrease frequency
		c_time = c_time + (error/(double)range_error)/1000;
	}
}

void normalize_samples(
	double data_buffer[][4], 
	int buffer_size, 
	int16_t sample_buffer[][3], 
	chrono::steady_clock::time_point time_buffer[], 
	double x_offset, 
	double y_offset, 
	double z_offset, 
	int scale_factor) {

	chrono::steady_clock::time_point start = time_buffer[0];

	for(int i =0;i<buffer_size;i++)	{
		data_buffer[i][0] = (double)chrono::duration_cast<chrono::microseconds>(time_buffer[i]-start).count()/1000;
		data_buffer[i][1] = (sample_buffer[i][0]+x_offset)/(double)scale_factor;
		data_buffer[i][2] = (sample_buffer[i][1]+y_offset)/(double)scale_factor;
		data_buffer[i][3] = (sample_buffer[i][2]+z_offset)/(double)scale_factor;
	}
}

void export_csv_data(double data_buffer[][4], int buffer_size, int index) {

	string filename =  "sample_csv_set_" + to_string(index) + ".csv";
	ofstream sample_file(filename);

	sample_file<<"time,Ax,Ay,Az"<<endl;

	for(int i=0;i<buffer_size;i++) {
		sample_file<<data_buffer[i][0]<<","<<data_buffer[i][1]<<","<<data_buffer[i][2]<<","<<data_buffer[i][3]<<endl;
	}

	sample_file.close();
}