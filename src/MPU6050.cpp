#include <MPU6050.h>

void MPU6050::begin() {
  	// power management
  	Wire.beginTransmission(MPU_ADDR);           // Start the communication by using address of MPU
  	Wire.write(0x6B);                           // Access the power management register
  	Wire.write(0b00000000);                     // Set sleep = 0
  	Wire.endTransmission();                     // End the communication

  	// configure gyro
  	Wire.beginTransmission(MPU_ADDR);
  	Wire.write(0x1B);                           // Access the gyro configuration register
  	Wire.write(0b00000000);						// +- 250 degrees per second
  	Wire.endTransmission();

  	// configure accelerometer
  	Wire.beginTransmission(MPU_ADDR);
  	Wire.write(0x1C);                           // Access the accelerometer configuration register
  	Wire.write(0b00000000);
  	Wire.endTransmission();  
}

void MPU6050::calibrateGyro() {
    long sumX = 0, sumY = 0, sumZ = 0;
    for (int i = 0; i < SAMPLES; i++) {
        Gyro g = readRawGyro();
        sumX = sumX + g.x;
        sumY = sumY + g.y;
        sumZ = sumY + g.z;
    }
    calli = {
        sumX / SAMPLES,
        sumY / SAMPLES,
        sumZ / SAMPLES,
    };
}

Gyro MPU6050::readRawGyro() {
  	Wire.beginTransmission(MPU_ADDR);                          // Start the communication by using address of MPU 
  	Wire.write(0x43);                                           // Access the starting register of gyro readings
  	Wire.endTransmission();
  	Wire.requestFrom(0b1101000, 6);                             // Request for 6 bytes from gyro registers (43 - 48)
  	while(Wire.available() < 6);                                // Wait untill all 6 bytes are available
    Gyro g = {
        x: (Wire.read()<<8 | Wire.read()) - calli.x,
        y: (Wire.read()<<8 | Wire.read()) - calli.y,
        z: (Wire.read()<<8 | Wire.read()) - calli.z,
    };
    return g;
}

Accel MPU6050::readAccel() {
  	Wire.beginTransmission(MPU_ADDR); 
  	Wire.write(0x3B); 
  	Wire.endTransmission();
  	Wire.requestFrom(MPU_ADDR, 6); 
  	while(Wire.available() < 6);	
	Accel a = {
		x: (Wire.read()<<8 | Wire.read()) / ACCEL_SENSITIVITY,
		y: (Wire.read()<<8 | Wire.read()) / ACCEL_SENSITIVITY,
		z: (Wire.read()<<8 | Wire.read()) / ACCEL_SENSITIVITY
	};
	return a;
}

// The caller should call this function in 250Hz
Angle MPU6050::calculateAngle() {
	Gyro g = readRawGyro();

	angle.x = angle.x + g.x * 0.000031; // = 1 / (131*250)
	angle.y = angle.y + g.y * 0.000031;
	angle.z = angle.z + g.z * 0.000031;

	Accel accel = readAccel();
	long accelTot = sqrt((accel.x * accel.x) + (accel.y * accel.y) + (accel.z * accel.z));

	float angleXaccel = angle.x;
	if (abs(accel.y) < accelTot) {
		angleXaccel = asin((float)accel.y / accelTot) * 57.296;
	}
	float angleYaccel = angle.y;
	if (abs(accel.x) < accelTot) {
		angleYaccel = asin((float)accel.x / accelTot) * -57.296;
	}

	// Too small coefficients (like 0.9996 and 0.0004) will not work
	angle.x = angle.x * 0.9 + angleXaccel * 0.1;
	angle.y = angle.y * 0.9 + angleYaccel * 0.1;

	return {
		x: angle.x,
		y: angle.y,
		z: angle.z
	};
}