#include <stdio.h>
#include <pigpio.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

// Power
#define POWER_CTL 	0x2D

//Output data rate
#define BW_RATE 	0x2C

//Data format
#define DATA_FORMAT 	0x31

//First data registry
#define DATAX0 		0x32

#define READ_BIT 	0x80
#define MULTI_BIT	0x40

const int maxFreq = 3200;
const int spiSpeed = 2000000;
const int maxSPIfreq = 100000;

int readBytes(int handle, char *data, int count) {
	data[0] |= READ_BIT;
	if(count > 1) data[0] |= MULTI_BIT;
	return spiXfer(handle, data, data, count);
}

int writeBytes(int handle, char *data, int count) {
	if(count > 1) data[0] |= MULTI_BIT;
	return spiWrite(handle, data, count);
}

int main(int argc, char *argv[]) {

	int tFreq = 512;
	int samples = tFreq * 4; //512Hz * 4
	int samplesMaxSPI = maxSPIfreq * 4;
	int h, bytes;
	char data[7];
	int16_t x, y, z;
	double tStart, tDuration, t;

	if(gpioInitialise() < 0) {
		printf("GPIO failed!");
		return 1;
	}

	h = spiOpen(0, spiSpeed, 3);

	data[0] = BW_RATE;
	data[1] = 0x0F;
	writeBytes(h, data, 2);

	data[0] = DATA_FORMAT;
	data[1] = 0x02;//8g
	writeBytes(h, data, 2);

	data[0] = POWER_CTL;
	data[1] = 0x08;
	writeBytes(h, data, 2);

	double dt = 1.0 / tFreq; //Time between samples

	//Start sampling

	double *at, *ax, *ay, *az;
	at = malloc(samples * sizeof(double));
	ax = malloc(samples * sizeof(double));
	ay = malloc(samples * sizeof(double));
	az = malloc(samples * sizeof(double));

	double *rt, *rx, *ry, *rz;
	rt = malloc(samplesMaxSPI * sizeof(double));
	rx = malloc(samplesMaxSPI * sizeof(double));
	ry = malloc(samplesMaxSPI * sizeof(double));
	rz = malloc(samplesMaxSPI * sizeof(double));

	tStart = time_time();

	int i;

	for(i=0; i<samplesMaxSPI; i++) {
		data[0] = DATAX0;
		bytes = readBytes(h, data, 7);
		if(bytes == 7) {
			x = (data[2]<<8) | (data[1]);
			y = (data[4]<<8) | (data[3]);
			z = (data[6]<<8) | (data[5]);
			t = time_time();
			rx[i] = (double) x;
			ry[i] = (double) y;
			rz[i] = (double) z;
			rt[i] = t-tStart;
		}
	}
	gpioTerminate();
	printf("%.4f time \n", t-tStart);
	printf("%d samples \n", i);
	return 0;
}

