#include "pid.h"
#include "stateControllerPID.h"

PidObject pidRollRate;
PidObject pidPitchRate;
PidObject pidYawRate;
PidObject pidRoll;
PidObject pidPitch;
PidObject pidYaw;

static int16_t rollOutput;
static int16_t pitchOutput;
static int16_t yawOutput;

static inline int16_t saturateSignedInt16(float in) {
    if (in > INT16_MAX)
        return INT16_MAX;
    else if (in < -INT16_MAX)
        return -INT16_MAX;
    else
        return (int16_t)in;
}

void attitudeControllerInit(const float updateDt) {
    pidInit(&pidRollRate, 0, PID_ROLL_RATE_KP, PID_ROLL_RATE_KI, PID_ROLL_RATE_KD, updateDt, 0, 0, false);
    pidInit(&pidPitchRate, 0, PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD, updateDt, 0, 0, false);
    pidInit(&pidYawRate, 0, PID_YAW_RATE_KP,   PID_YAW_RATE_KI,   PID_YAW_RATE_KD, updateDt, 0, 0, false);

    pidSetIntegralLimit(&pidRollRate,  PID_ROLL_RATE_INTEGRATION_LIMIT);
    pidSetIntegralLimit(&pidPitchRate, PID_PITCH_RATE_INTEGRATION_LIMIT);
    pidSetIntegralLimit(&pidYawRate,   PID_YAW_RATE_INTEGRATION_LIMIT);

    pidInit(&pidRoll,  0, PID_ROLL_KP,  PID_ROLL_KI,  PID_ROLL_KD,  updateDt, 0, 0, false);
    pidInit(&pidPitch, 0, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, updateDt, 0, 0, false);
    pidInit(&pidYaw,   0, PID_YAW_KP,   PID_YAW_KI,   PID_YAW_KD,   updateDt, 0, 0, false);

    pidSetIntegralLimit(&pidRoll,  PID_ROLL_INTEGRATION_LIMIT);
    pidSetIntegralLimit(&pidPitch, PID_PITCH_INTEGRATION_LIMIT);
    pidSetIntegralLimit(&pidYaw,   PID_YAW_INTEGRATION_LIMIT);
}

void attitudeControllerCorrectAttitudePID(
       float eulerRollActual, float eulerPitchActual, float eulerYawActual,
       float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
       float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired) {
  	pidSetDesired(&pidRoll, eulerRollDesired);
  	*rollRateDesired = pidUpdate(&pidRoll, eulerRollActual, true);

  	// Update PID for pitch axis
  	pidSetDesired(&pidPitch, eulerPitchDesired);
  	*pitchRateDesired = pidUpdate(&pidPitch, eulerPitchActual, true);

  	// Update PID for yaw axis
  	float yawError;
  	yawError = eulerYawDesired - eulerYawActual;
  	if (yawError > 180.0f)
  	  	yawError -= 360.0f;
  	else if (yawError < -180.0f)
  	  	yawError += 360.0f;
  	pidSetError(&pidYaw, yawError);
  	*yawRateDesired = pidUpdate(&pidYaw, eulerYawActual, false);
}

void attitudeControllerCorrectRatePID(
       float rollRateActual, float pitchRateActual, float yawRateActual,
       float rollRateDesired, float pitchRateDesired, float yawRateDesired) {
    pidSetDesired(&pidRollRate, rollRateDesired);
    rollOutput = saturateSignedInt16(pidUpdate(&pidRollRate, rollRateActual, true));

    pidSetDesired(&pidPitchRate, pitchRateDesired);
    pitchOutput = saturateSignedInt16(pidUpdate(&pidPitchRate, pitchRateActual, true));

    pidSetDesired(&pidYawRate, yawRateDesired);
    yawOutput = saturateSignedInt16(pidUpdate(&pidYawRate, yawRateActual, true));
}

void attitudeControllerGetActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw) {
    *roll = rollOutput;
    *pitch = pitchOutput;
    *yaw = yawOutput;
}

void attitudeControllerResetAllPID(void) {
    pidReset(&pidRoll);
    pidReset(&pidPitch);
    pidReset(&pidYaw);
    pidReset(&pidRollRate);
    pidReset(&pidPitchRate);
    pidReset(&pidYawRate);
}
