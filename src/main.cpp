#include <Arduino.h>
#include <Wire.h>

#define MPU_ADDR 0b1101000
#define SENSITIVITY_FACTOR 16384.0
#define REFRESH_TIME 4000 // 4000 us
#define CALIBRATION_ITERATIONS 2000

#define P_GAIN 1.3
#define I_GAIN 0.04
#define D_GAIN 18.0
#define MAX_PID_ANGLE 85

#define M1_MOTOR 11
#define M2_MOTOR 10

#define BASE_SPEED 50 // A value between 0 - 255

#define X_REF 0.0

float accelX, accelY, accelZ;

long gyroXCalli = 0, gyroYCalli = 0, gyroZCalli = 0;
long gyroXPresent = 0, gyroYPresent = 0, gyroZPresent = 0;
long gyroXPast = 0, gyroYPast = 0, gyroZPast = 0;
float rotX, rotY, rotZ;

float angelX = 0, angelY = 0, angelZ = 0;
float angelXError = 0, angelYError = 0, angelZError = 0;

float pidAngleXI = 0;
float prevAngleXError = 0;

float pidAngleX = 0, pidAngleY = 0;

long timePast = 0;
long timePresent = 0;

unsigned long loopTimer;

void setUpMPU();
void accelData();
void getGyroValues();
void angularData();
void angularDataCalibration();
void PID();
void updateMotorSpeed();
void print();

void setup() {
	// Initiate
	Serial.begin(57600);
	setUpMPU();
	// calibrate
	angularDataCalibration();
	delay(2000);
	loopTimer = micros();  
}

void setUpMPU() {
	digitalWrite(LED_BUILTIN, LOW);
  	// power management
  	Wire.beginTransmission(MPU_ADDR);          // Start the communication by using address of MPU
  	Wire.write(0x6B);                           // Access the power management register
  	Wire.write(0b00000000);                     // Set sleep = 0
  	Wire.endTransmission();                     // End the communication

  	// configure gyro
  	Wire.beginTransmission(MPU_ADDR);
  	Wire.write(0x1B);                           // Access the gyro configuration register
  	Wire.write(0b00000000);
  	Wire.endTransmission();

  	// configure accelerometer
  	Wire.beginTransmission(MPU_ADDR);
  	Wire.write(0x1C);                           // Access the accelerometer configuration register
  	Wire.write(0b00000000);
  	Wire.endTransmission();  
}

void angularDataCalibration() {
	long x = 0, y = 0, z = 0;
    for (int i = 0; i < CALIBRATION_ITERATIONS; i++) {
      	getGyroValues();
      	x = x + gyroXPresent;
      	y = y + gyroYPresent;
      	z = z + gyroZPresent;
		delay(1);
    }
    gyroXCalli = x / CALIBRATION_ITERATIONS;
    gyroYCalli = y / CALIBRATION_ITERATIONS;
    gyroZCalli = z / CALIBRATION_ITERATIONS;
}

int i = 0;
void loop() {
	accelData();
	angularData();

	// calculate PID
	PID();

	// update PWM 
	updateMotorSpeed();
	
    while(micros() - loopTimer < REFRESH_TIME) {}
	loopTimer = micros();    
	// print if necessary
	if (i == 125) {
		i = 0;
		print();
	} else {
		i = i + 1;
	}
}

void accelData() {
  	Wire.beginTransmission(MPU_ADDR); 
  	Wire.write(0x3B); 
  	Wire.endTransmission();
  	Wire.requestFrom(MPU_ADDR, 6); 
  	while(Wire.available() < 6) {
	}
  	accelX = (Wire.read()<<8 | Wire.read()) / SENSITIVITY_FACTOR;
  	accelY = (Wire.read()<<8 | Wire.read()) / SENSITIVITY_FACTOR;
  	accelZ = (Wire.read()<<8 | Wire.read()) / SENSITIVITY_FACTOR;
}

void angularData() {
  	gyroXPast = gyroXPresent;
  	gyroYPast = gyroYPresent;
  	gyroZPast = gyroZPresent;
  	timePast = timePresent;
  	timePresent = millis();

  	getGyroValues();
  	// angelX = angelX + ((timePresent - timePast)*(gyroXPresent + gyroXPast)) * 0.00000382; // angel around X axis
  	// angelY = angelY + ((timePresent - timePast)*(gyroYPresent + gyroYPast)) * 0.00000382;
  	// angelZ = angelZ + ((timePresent - timePast)*(gyroZPresent + gyroZPast)) * 0.00000382;

	angelX = angelX + gyroXPresent * 0.000031; // 0.000031
	angelY = angelY + gyroYPresent * 0.000031; // 0.000031
	angelZ = angelZ + gyroZPresent * 0.000031; // 0.000031

	long accelTot = sqrt((accelX * accelX) + (accelY * accelY) + (accelZ * accelZ));

	float angleXaccel = angelX;
	if (abs(accelY) < accelTot) {
		angleXaccel = asin((float)accelY / accelTot) * 57.296;
	}
	float angleYaccel = angelY;
	if (abs(accelX) < accelTot) {
		angleYaccel = asin((float)accelX / accelTot) * -57.296;
	}

	// angelX = angelX * 0.9996 + angleXaccel * 0.0004; // Too small coefficients (like 0.0004) will not work
	// angelY = angelY * 0.9996 + angleYaccel * 0.0004;
	angelX = angelX * 0.9 + angleXaccel * 0.1;
	angelY = angelY * 0.9 + angleYaccel * 0.1;
}

void getGyroValues() {
  	Wire.beginTransmission(0b1101000);                          // Start the communication by using address of MPU 
  	Wire.write(0x43);                                           // Access the starting register of gyro readings
  	Wire.endTransmission();
  	Wire.requestFrom(0b1101000, 6);                             // Request for 6 bytes from gyro registers (43 - 48)
  	while(Wire.available() < 6);                                // Wait untill all 6 bytes are available
  	gyroXPresent = (Wire.read()<<8 | Wire.read()) - gyroXCalli;                  
  	gyroYPresent = (Wire.read()<<8 | Wire.read()) - gyroYCalli;                 
  	gyroZPresent = (Wire.read()<<8 | Wire.read()) - gyroZCalli;                
}

void PID() {
	// calculate errors
	// expected angular velocity and displacement is zero, since in first iteration
	// it is only expected to balance in the air.
	float error = angelX - X_REF;
	pidAngleXI = pidAngleXI + I_GAIN * error;
	if (pidAngleXI > MAX_PID_ANGLE) {
		pidAngleXI = MAX_PID_ANGLE;
	} else if (pidAngleXI < MAX_PID_ANGLE * -1) {
		pidAngleXI = MAX_PID_ANGLE * -1;
	}
	pidAngleX = P_GAIN * error + pidAngleXI + D_GAIN * (error - prevAngleXError);
	if (pidAngleX > MAX_PID_ANGLE) {
		pidAngleX = MAX_PID_ANGLE;
	} else if (pidAngleX < MAX_PID_ANGLE * -1) {
		pidAngleX = MAX_PID_ANGLE * -1;
	}
	prevAngleXError = error;

	// pid_error_temp = gyro_roll_input - pid_roll_setpoint;
	// pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
	// if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
	// else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

	// pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
	// if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
	// else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

	// pid_last_roll_d_error = pid_error_temp;
}

void updateMotorSpeed() {
	int m1 = BASE_SPEED - pidAngleX;
	int m2 = BASE_SPEED + pidAngleX;
	analogWrite(M1_MOTOR, m1);
	analogWrite(M2_MOTOR, m2);
}

void print() {
	Serial.print("angleX = ");
	Serial.println(angelX);
	if (angelX > 0) {
		digitalWrite(LED_BUILTIN, HIGH);
		delay(100);
		digitalWrite(LED_BUILTIN, LOW);
	}
}