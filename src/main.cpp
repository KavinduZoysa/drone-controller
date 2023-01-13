#include <Arduino.h>
#include <MPU6050.h>

#define REFRESH_TIME 4000 // 4000 us

#define P_GAIN 1.3
#define I_GAIN 0.04
#define D_GAIN 18.0
#define MAX_PID_ANGLE 85

#define M1_MOTOR 11
#define M2_MOTOR 10

#define BASE_SPEED 50 // A value between 0 - 255

#define X_REF 0.0

MPU6050 mpu;
Angle angle;

float pidAngleXI = 0;
float prevAngleXError = 0;

float pidAngleX = 0, pidAngleY = 0;

unsigned long loopTimer;

void PID();
void updateMotorSpeed();
void print();

void setup() {
	Serial.begin(57600);
	mpu.begin();
	mpu.calibrateGyro();
	delay(2000);
	loopTimer = micros();  
}

int i = 0;
void loop() {
	angle = mpu.calculateAngle();

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

void PID() {
	// calculate errors
	// expected angular velocity and displacement is zero, since in first iteration
	// it is only expected to balance in the air.
	float error = angle.x - X_REF;
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
	Serial.print("angle.x = ");
	Serial.println(angle.x);
	if (angle.x > 0) {
		digitalWrite(LED_BUILTIN, HIGH);
		delay(100);
		digitalWrite(LED_BUILTIN, LOW);
	}
}