#include "types.h"
#include "controllerPID.h"
#include "stateControllerPID.h"

static attitude_t attitudeDesired;
static attitude_t rateDesired;

void stateControllerInit(const float updateDt) {
  	attitudeControllerInit(updateDt);
}

void stateController(control_t *control, setpoint_t *setpoint, const sensor_t *sensors, const state_t *state) {
    attitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
                                attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
                                &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);

    attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z, rateDesired.roll, rateDesired.pitch, rateDesired.yaw);

    attitudeControllerGetActuatorOutput(&control->roll, &control->pitch, &control->yaw);

    control->yaw = -control->yaw;
    // In original code this is done in special frequency

    if (control->thrust == 0) {
        control->thrust = 0;
        control->roll = 0;
        control->pitch = 0;
        control->yaw = 0;

        attitudeControllerResetAllPID();

        // Reset the calculated YAW angle for rate control
        attitudeDesired.yaw = state->attitude.yaw;
    }
}