import smbus
import time
import csv
#--------------------------------------------------------
# MPU6050 Registers and addresses
#--------------------------------------------------------
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
DLPF_CFG = 0x26
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE = 0x38
FIFO_EN = 0x23
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

#MPU6050 device address
Device_Address = 0x68

bus = smbus.SMBus(1)

#--------------------------------------------------------
# Measured offsets
#--------------------------------------------------------
AX_OFFSET = 0
AY_OFFSET = 0
AZ_OFFSET = 0

#--------------------------------------------------------
# Calibration parameters
#--------------------------------------------------------
#amount used to calculate average
buffer_size = 1000
#accelerometer error allowed
accel_error_range = 10

#--------------------------------------------------------
# Config MPU function
#--------------------------------------------------------
def MPU_INIT():
	#write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 0)
	
	#write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	
	#write to configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	
	#write to gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	
	#write to accel configuration register
	bus.write_byte_data(Device_Address, ACCEL_CONFIG, 24)
	
	#write to interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)
	
	#write to DLPF
	bus.write_byte_data(Device_Address, DLPF_CFG, 0)
	
	#write to FIFO_EN
	bus.write_byte_data(Device_Address, FIFO_EN, 8)

#--------------------------------------------------------
# Read MPU raw values LSB/g
#--------------------------------------------------------
def read_raw_data(addr):

	#Accelerometer and Gyro value are 16-bit
	high = bus.read_byte_data(Device_Address, addr)
	low = bus.read_byte_data(Device_Address, addr+1)
	
	#concatenate higher and lower value
	value = ((high << 8) | low)
	print(value)
	return value

def signed_data(value):

	#to get signed value from mpu6050
	if(value>32768):
		value = value - 65536

	return value

#--------------------------------------------------------
# Calculate average sample function
#--------------------------------------------------------
def AVG_DATA():

	avg_ax = avg_ay = avg_az = 0

	i = buff_ax = buff_ay = buff_az = 0
	
	while i<(buffer_size+101):
		ax = signed_data(read_raw_data(ACCEL_XOUT_H)) + AX_OFFSET
		ay = signed_data(read_raw_data(ACCEL_YOUT_H)) + AY_OFFSET
		az = signed_data(read_raw_data(ACCEL_ZOUT_H)) + AZ_OFFSET

		if(i>100 and  i<=(buffer_size+100)):
			buff_ax = buff_ax + ax
			buff_ay = buff_ay + ay
			buff_az = buff_az + az

		if(i==(buffer_size+100)):
			avg_ax = buff_ax/buffer_size
			avg_ay = buff_ay/buffer_size
			avg_az = buff_az/buffer_size

		i = i + 1
		#gap between samples
		time.sleep(0.002)

	return avg_ax, avg_ay, avg_az

#--------------------------------------------------------
# Calibrate function
#--------------------------------------------------------
def CALIBRATE():

	global AX_OFFSET
	global AY_OFFSET
	global AZ_OFFSET

	c_ax_offset = c_ay_offset = c_az_offset = 0
	avg_ax, avg_ay, avg_az = AVG_DATA()

	# Initial offset
	c_ax_offset = -avg_ax/accel_error_range
	c_ay_offset = -avg_ay/accel_error_range
	c_az_offset = (2048.0 - avg_az)/accel_error_range

	while True:
		ready = 0
		AX_OFFSET = c_ax_offset
		AY_OFFSET = c_ay_offset
		AZ_OFFSET = c_az_offset

		# print(AX_OFFSET, AY_OFFSET, AZ_OFFSET)
		
		avg_ax, avg_ay, avg_az = AVG_DATA()

		if(abs(avg_ax)<=accel_error_range):
			ready+=1
		else:
			c_ax_offset = c_ax_offset - avg_ax/accel_error_range

		if(abs(avg_ay)<=accel_error_range):
			ready+=1
		else:
			c_ay_offset = c_ay_offset - avg_ay/accel_error_range

		if(abs(2048-avg_az)<=accel_error_range):
			ready+=1
		else:
			c_az_offset = c_az_offset + (2048-avg_az)/accel_error_range

		if(ready==3):
			break

#--------------------------------------------------------
#   Collect Data
#--------------------------------------------------------
def get_samples(size):

	samples = []
	i=0
	
	while i<size:

		acc_x = read_raw_data(ACCEL_XOUT_H)
		acc_y = read_raw_data(ACCEL_YOUT_H)
		acc_z = read_raw_data(ACCEL_ZOUT_H)

		samples.append([acc_x, acc_y, acc_z, time.time()])
		
		i+=1

	return samples

#--------------------------------------------------------
#   Normalize Data
#--------------------------------------------------------
def normalize_samples(samples):

	start_time = samples[0][3]
	normalized_samples = []

	norm_x_list = []
	norm_y_list = []
	norm_z_list = []
	norm_time_list = []

	for sample in samples:
		norm_x = (signed_data(sample[0]) + AX_OFFSET)/2048.0
		norm_y = (signed_data(sample[1]) + AY_OFFSET)/2048.0
		norm_z = (signed_data(sample[2]) + AZ_OFFSET)/2048.0
		norm_time = sample[3]-start_time

		norm_x_list.append(norm_x)
		norm_y_list.append(norm_y)
		norm_z_list.append(norm_z)
		norm_time_list.append(norm_time)

		normalized_samples.append([norm_x, norm_y, norm_z, norm_time])

	return normalized_samples

#--------------------------------------------------------
#   Export CSV Data
#--------------------------------------------------------
def export_csv_data(data, sample_set):
	
	with open('sample_csv_file_set'+str(sample_set), 'w') as csv_file:
		fieldNames = ['Ax','Ay','Az','time']
		
		row_list = data.insert(0,fieldNames)

		writer = csv.writer(csv_file)
		writer.writerows(data)

#--------------------------------------------------------
#	Execution
#--------------------------------------------------------
MPU_INIT()

input("Press enter to continue...")
print("Calibrating...")
CALIBRATE()

key = input("Press enter to continue...")
sample_set = 0
while(key!='s'):

	print("Reading data...")

	samples = get_samples(1000)

	print("Normalizing...")
	normalized_samples = normalize_samples(samples)

	sample_set+=1
	export_csv_data(normalized_samples, sample_set)

	key = input("Press enter to continue or s to stop...")
