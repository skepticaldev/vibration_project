#include <stdio.h>
#include <sys/types.h>
#include <time.h>
#include <chrono>
#include <iostream>
#include <tuple>
#include <fstream>
#include <string>
#include <thread>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include <jsoncpp/json/json.h>

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
#define RANGE_ERROR 2.5

using namespace std;

//--------------------------
// Function declarations
//--------------------------

// MPU config function
void mpu_init(int file);

// Write config values to registers
void write_byte_data(int file, __u8 reg, __u8 byte);

// Read MPU raw values LSB/g
int16_t read_raw_data(int file, __u8 reg);

// Calculate average sample
tuple <double,double,double> avg_data(int buffer_size, double wait_time, int file, double X_OFFSET, double Y_OFFSET, double Z_OFFSET);

// Calibrate function
tuple <double, double, double> calibrate(int file, int range_error, double wait_time, int buffer_size, bool trace);

// Collect Data
void collect_samples(
	int file, 
	int size, 
	int16_t sample_buffer[][3],
	chrono::steady_clock::time_point time_buffer[], 
	//seconds
	double wait_time);

// Export data to csv file
void export_csv_data(double data_buffer[][4], int buffer_size, int filename);

// Calculte wait time between samples to achieve desired frequency
double calculate_wait_time(int file, int buffer_size, int target_frequency, int frequency_range_error, bool trace);

// Normalize samples
void normalize_samples(
	double data_buffer[][4], 
	int buffer_size, 
	int16_t sample_buffer[][3], 
	chrono::steady_clock::time_point time_buffer[], 
	double x_offset, 
	double y_offset, 
	double z_offset, 
	int scale_factor);

// Load Config
Json::Value load_config(string filename);

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

	// Load config.json file
	cout<<"Reading config file..."<<endl;
	Json::Value config = load_config("config.json");

	// MPU configuration
	cout<<"Initializing..."<<endl;
	mpu_init(file_i2c);

	int buffer_size = config["BUFFER"].asUInt();

	double x_offset = 0, y_offset = 0, z_offset = 0;
	
	cout<<"Calculating wait time..."<<endl;
	double wait_time = calculate_wait_time(file_i2c, 1000, config["FREQUENCY"].asUInt(), 1, config["TRACE"].asBool());

	//Calibration
	cout<<"Calibrating..."<<endl;
	tie(x_offset, y_offset, z_offset) = calibrate(file_i2c, config["CALIBRATION_RANGE_ERROR"].asDouble(), wait_time, config["CALIBRATION_BUFFER"].asUInt(), config["TRACE"].asBool());

	config["X_OFFSET"] = x_offset;
	config["Y_OFFSET"] = y_offset;
	config["Z_OFFSET"] = z_offset;

	ofstream config_file("config.json");
	
	Json::StyledWriter styled;
	string sStyled = styled.write(config);
	Json::StyledStreamWriter styledStream;
	config_file<<sStyled;
	config_file.close();
	
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
		collect_samples(file_i2c, buffer_size, sample_buffer, time_buffer, wait_time);

		//for(int i=0; i<buffer_size;i++){
		//	cout<<""<< (double)chrono::duration_cast<chrono::microseconds>(time_buffer[i]-time_buffer[0]).count()/1000
		//	<<""<<sample_buffer[i][0]<<" "<<sample_buffer[i][1]<<" "<<sample_buffer[i][2]<<endl;
		//}

		double data_buffer[buffer_size][4];

		cout<<"Normalizing samples..."<<endl;
		normalize_samples(data_buffer, buffer_size, sample_buffer, time_buffer, x_offset, y_offset, z_offset, 128);

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

Json::Value load_config(string filename) {

	ifstream config_file(filename);
	Json::Reader reader;
	Json::Value config;

	bool b = reader.parse(config_file, config);
	
	if(!b) {
		cout<<"Error reading configuration file!"<<endl;
		cout<<reader.getFormattedErrorMessages()<<endl;
		exit(1);
	}

	return config;
}

void mpu_init(int file) {

	//power
	write_byte_data(file, POWER_CLT, 0x08);

	//write to Output data rate
	write_byte_data(file, BW_RATE, 0x0c);

	//write to data format
	write_byte_data(file, DATA_FORMAT, 0x01);
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
	double wait_time) {

	char buf[6];

	struct timespec req = {0};
	req.tv_sec = 0;
	req.tv_nsec = wait_time * 1000000000L;

	for (int i=0;i<size;i++) {
		buf[0] = 0x32;
		
		if((write(file, buf, 1)) != 1) {
			printf("Error writing to i2c slave\n");
			exit(1);
		};

		if((read(file, buf, 6)) != 6) {
			printf("Unable to read from slave\n");
			exit(1);
		} else {
			time_buffer[i] = chrono::steady_clock::now();
			sample_buffer[i][0] = ((int16_t) buf[1]<<8 | (int16_t) buf[0]);
			sample_buffer[i][1] = ((int16_t) buf[3]<<8 | (int16_t) buf[2]);
			sample_buffer[i][2] = ((int16_t) buf[5]<<8 | (int16_t) buf[4]);

			nanosleep(&req, (struct timespec *) NULL);
		};
	}
}

tuple <double,double,double> avg_data(int buffer_size, int file, double wait_time, double X_OFFSET=0, double Y_OFFSET=0, double Z_OFFSET=0){
	
	double avg_ax = 0, avg_ay = 0, avg_az = 0;
	
	double buff_ax = 0, buff_ay = 0, buff_az = 0;
	
	int i = 0;

	struct timespec req = {0};
	req.tv_sec = 0;
	req.tv_nsec = wait_time * 1000000000L;

	char buf[6];

	while (i<buffer_size+101) {
		
		if(i>100 && i<=(buffer_size+100)) {
			buf[0] = 0x32;
			
			if((write(file , buf, 1)) != 1) {
				printf("Error writing to i2c slave\n");
				exit(1);
			};

			if((read(file, buf, 6)) != 6) {
				printf("Unable to read from slave\n");
				exit(1);
			} else {
				int16_t data_x = ((int16_t) buf[1]<<8 | (int16_t) buf[0]);
				int16_t data_y = ((int16_t) buf[3]<<8 | (int16_t) buf[2]);
				int16_t data_z = ((int16_t) buf[5]<<8 | (int16_t) buf[4]);
				
				// cout<<"X: "<<data_x<<" Y: "<<data_y<<" Z: "<<data_z<<endl;

				buff_ax = buff_ax + X_OFFSET + (double) data_x;
				buff_ay = buff_ay + Y_OFFSET + (double) data_y;
				buff_az = buff_az + Z_OFFSET + (double) data_z;
				
				//cout<<"buff_ax: "<<buff_ax<<" buff_ay: "<<buff_ay<<" buff_az: "<<buff_az<<endl;

				nanosleep(&req, (struct timespec *) NULL);
			}
		}

		if (i==(buffer_size+100)) {
			avg_ax = buff_ax/(double) buffer_size;
			avg_ay = buff_ay/(double) buffer_size;
			avg_az = buff_az/(double) buffer_size;
		}

		i+=1;
	}
	// cout<<"avg_x: "<<avg_ax<<" avg_y: "<<avg_ay<<" avg_z: "<<avg_az<<endl;

	return make_tuple(avg_ax, avg_ay, avg_az);
}

tuple <double, double, double> calibrate(int file, int range_error, double wait_time, int buffer_size, bool trace){

	double avg_x, avg_y,avg_z;
	double x_offset = 0, y_offset = 0, z_offset = 0;

	tie(avg_x, avg_y, avg_z) = avg_data(buffer_size, file, wait_time, 0, 0, 0);

	x_offset = -avg_x/range_error;
	y_offset = -avg_y/range_error;
	z_offset = -avg_z/range_error;

	while (true) {
		int ready = 0;

		tie(avg_x, avg_y, avg_z) = avg_data(buffer_size, file, wait_time, x_offset, y_offset, z_offset);

		if(trace) {
			cout<<"X_OFFSET: "<<x_offset<<" Y_OFFSET: "<<y_offset<<" Z_OFFSET: "<<z_offset<<endl;
		}

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
			cout<<"Offsets XYZ: " << x_offset<<" "<<y_offset<<" "<<z_offset<<endl;
			return make_tuple(x_offset, y_offset, z_offset);
		}
	}
}

double calculate_wait_time(int file, int buffer_size, int target_frequency, int frequency_range_error, bool trace) {
	
	//wait time in seconds
	double wait_time = 0.0;

	while(true) {

		//Temp buffer to simulate data sampling
		int16_t temp_sample_buffer[buffer_size][3];
		
		//Temp buffer to simulate time sampling
		chrono::steady_clock::time_point temp_time_buffer[buffer_size];
		
		struct timespec req = {0};
		req.tv_sec = 0;
		req.tv_nsec = wait_time * 1000000000L;

		//Start time to calculate elapsed time
		chrono::steady_clock::time_point start = chrono::steady_clock::now();

		char buf[6];

		for(int i=0;i<buffer_size;i++) {
			buf[0] = 0x32;
			
			if((write(file, buf, 1)) != 1) {
				printf("Error writing to i2c slave\n");
				exit(1);
			};

			if((read(file, buf, 6)) != 6) {
				printf("Unable to read from slave\n");
				exit(1);
			} else {
				temp_time_buffer[i] = chrono::steady_clock::now();
				temp_sample_buffer[i][0] = ((int16_t) buf[1]<<8 | (int16_t) buf[0]);
				temp_sample_buffer[i][1] = ((int16_t) buf[3]<<8 | (int16_t) buf[2]);
				temp_sample_buffer[i][2] = ((int16_t) buf[5]<<8 | (int16_t) buf[4]);

				nanosleep(&req, (struct timespec *) NULL);
			}
		}
		
		// Calculate acquisition time
		double acquisition_time = (double) chrono::duration_cast<chrono::nanoseconds>(temp_time_buffer[buffer_size-1]-start).count();
		
		// Calculate acquisition period in nanoseconds
		double acquisition_period = acquisition_time/(double) buffer_size;

		// Calculate acquisition frequency (convert acquisition period to seconds)
		double acquisition_frequency = 1*1000000000/(double) acquisition_period;

		if(trace) {
			cout<<"Frequency: "<<acquisition_frequency<<" HZ"<<" Wait time: "<<wait_time<<" s"<<endl;
		}

		if((wait_time==0.0) && (acquisition_frequency<target_frequency)) {
			cout<<"Unable to reach "<<target_frequency<<"Hz acquisition frequency"<<endl;
			return wait_time;
		}
		
		if(abs(acquisition_frequency-target_frequency)<=frequency_range_error) {
			cout<<"Acquisition frequency: "<<acquisition_frequency<<" HZ"<<endl;
			return wait_time;
		}
		
		// Calculate target period
		double target_period_time = 1/(double) target_frequency;

		double time_error = target_period_time -  acquisition_period/(double) 1000000000;

		wait_time = wait_time + time_error/((double) frequency_range_error);
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

	string filename =  "samples_" + to_string(index) + ".csv";
	ofstream sample_file(filename);

	sample_file<<"time,Ax,Ay,Az"<<endl;

	for(int i=0;i<buffer_size;i++) {
		sample_file<<data_buffer[i][0]<<","<<data_buffer[i][1]<<","<<data_buffer[i][2]<<","<<data_buffer[i][3]<<endl;
	}

	sample_file.close();
}
