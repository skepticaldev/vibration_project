import smbus
import time

#MPU6050 Registers and address

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

#Calculated offsets
AX_OFFSET = 0
AY_OFFSET = 0
AZ_OFFSET = 0


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

bus = smbus.SMBus(1)
Device_Address = 0x68 #MPU6050 device address

MPU_INIT()

#Calibrate section
#
#
buffer_size = 1000 #reading amount used to calculate average
accel_deadzone = 8 #accelerometer error allowed

mean_ax = mean_ay = mean_az = state = 0
c_ax_offset = c_ay_offset = c_az_offset = 0

def mean_sensors():
	global mean_ax
	global mean_ay
	global mean_az

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
			mean_ax = buff_ax/buffer_size
			mean_ay = buff_ay/buffer_size
			mean_az = buff_az/buffer_size

		i = i + 1
		time.sleep(0.002)

	print(mean_ax, mean_az, mean_ay)

def calibration():
	global c_ax_offset
	global c_ay_offset
	global c_az_offset

	global AX_OFFSET
	global AY_OFFSET
	global AZ_OFFSET

	c_ax_offset = -mean_ax/8
	c_ay_offset = -mean_ay/8
	c_az_offset = (2048.0 - mean_az)/8

	while True:
		ready = 0
		AX_OFFSET = c_ax_offset
		AY_OFFSET = c_ay_offset
		AZ_OFFSET = c_az_offset
		
		mean_sensors()
		print("...")

		if(abs(mean_ax)<=accel_deadzone):
			ready+=1
			print("x ready...")
		else:
			c_ax_offset = c_ax_offset - mean_ax/accel_deadzone

		if(abs(mean_ay)<=accel_deadzone):
			ready+=1
			print("y ready...")
		else:
			c_ay_offset = c_ay_offset - mean_ay/accel_deadzone

		if(abs(2048-mean_az)<=accel_deadzone):
			ready+=1
			print("z ready...")
		else:
			c_az_offset = c_az_offset + (2048-mean_az)/accel_deadzone

		if(ready==3):
			break
#
#
#
print("Calibrating...")
mean_sensors()


calibration()

print("Calibration values...")
print(mean_ax)
print(mean_ay)
print(mean_az)
print(c_ax_offset)
print(c_ay_offset)
print(c_az_offset)

print("Reading data...")

start_time = time.time()
while True:
	
	#Read Accelerometer raw value
	acc_x = read_raw_data(ACCEL_XOUT_H, AX_OFFSET)
	acc_y = read_raw_data(ACCEL_YOUT_H, AY_OFFSET)
	acc_z = read_raw_data(ACCEL_ZOUT_H, AZ_OFFSET)

	#Full scale range +/- 250 degree/C as per sensitivity scale factor
	Ax = acc_x/2048.0
	Ay = acc_y/2048.0
	Az = acc_z/2048.0

	print(Ax, Ay, Az)
	#print(acc_x,acc_y,acc_z)

print(len(sample_times))
temp = start_time
#for st in sample_times:
#	print((st-temp)*1000)
#	temp = st
