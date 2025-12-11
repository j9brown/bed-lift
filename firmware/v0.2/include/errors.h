/**
 * @file
 * @brief Specific error codes that describe the state of the bed lift.
 */

#define BED_ERROR_BAD_STATE (-10000)  /* The bed components are in an unknown or unexpected state. */

#define SPAN_ERROR_FAULT (-10100)    /* Motor driver reported a fault. */
#define SPAN_ERROR_DESYNC (-10101)   /* The actuators lost synchronization. */
#define SPAN_ERROR_DRIVER (-10102)   /* I/O error communicating with the motor driver. */
#define SPAN_ERROR_TIMEOUT (-10103)  /* Operation timed out because poll wasn't called often enough. */
#define SPAN_ERROR_NOT_HOME (-10104) /* The actuators did not stall at their home position as expected. */

#define LIFT_ERROR_FAULT (-10200)   /* Motor driver reported a fault. */
#define LIFT_ERROR_DESYNC (-10201)  /* The actuators lost synchronization. */
#define LIFT_ERROR_DRIVER (-10202)  /* I/O error communicating with the motor driver. */
#define LIFT_ERROR_TIMEOUT (-10203) /* Operation timed out because poll wasn't called often enough. */
#define LIFT_ERROR_STALL (-10204)   /* Lift stationary when it should be moving or the hall sensors are not working correctly. */
