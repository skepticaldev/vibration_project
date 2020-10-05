import smbus
import time

#--------------------------------------------------------
#						MPU6050 Registers and adresses
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
#	Measured offsets
#--------------------------------------------------------
AX_OFFSET = 0
AY_OFFSET = 0
AZ_OFFSET = 0

#--------------------------------------------------------
#	Calibration parameters
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
def read_raw_data(addr, offset):

	#Accelerometer and Gyro value are 16-bit
	high = bus.read_byte_data(Device_Address, addr)
	low = bus.read_byte_data(Device_Address, addr+1)
	
	#concatenate higher and lower value
	value = ((high << 8) | low)

	#to get signed value from mpu6050
	if(value > 32768):
		value = value - 65536
	
	return value + offset


#--------------------------------------------------------
#	Calculate samples average
#--------------------------------------------------------
def AVG_DATA():

	avg_ax = avg_ay = avg_az = 0

	i = buff_ax = buff_ay = buff_az = 0
	
	while i<(buffer_size+101):
		ax = read_raw_data(ACCEL_XOUT_H, AX_OFFSET)
		ay = read_raw_data(ACCEL_YOUT_H, AY_OFFSET)
		az = read_raw_data(ACCEL_ZOUT_H, AZ_OFFSET)

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

	return avg_ax, avg_az, avg_ay

def CALIBRATE():

	global AX_OFFSET
	global AY_OFFSET
	global AZ_OFFSET

	c_ax_offset = c_ay_offset = c_az_offset = 0
	avg_ax, avg_ay, avg_az = AVG_DATA()

	c_ax_offset = -avg_ax/accel_error_range
	c_ay_offset = -avg_ay/accel_error_range
	c_az_offset = (2048.0 - avg_az)/accel_error_range

	while True:
		ready = 0
		AX_OFFSET = c_ax_offset
		AY_OFFSET = c_ay_offset
		AZ_OFFSET = c_az_offset

		print(AX_OFFSET, AY_OFFSET, AZ_OFFSET)
		
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
#	Execution
#--------------------------------------------------------

MPU_INIT()

print("Calibrating...")
CALIBRATE()

print("Reading data...")

while True:
	
	#Read Accelerometer raw value
	acc_x = read_raw_data(ACCEL_XOUT_H, AX_OFFSET)
	acc_y = read_raw_data(ACCEL_YOUT_H, AY_OFFSET)
	acc_z = read_raw_data(ACCEL_ZOUT_H, AZ_OFFSET)

	#sensitivity scale factor
	Ax = acc_x/2048.0
	Ay = acc_y/2048.0
	Az = acc_z/2048.0

	print(Ax, Ay, Az)

