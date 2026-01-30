/**
 * @file
 * @brief Specific error codes that describe the state of the bed lift.
 *
 * These error codes share space with the negative errno set of values and are constrained
 * to fit into 1 byte when reported to external systems so they start at -255 and work
 * their way up.  The color of the blink error is related to the error category and the
 * index determines the number of indicator blinks for that error.
 */

#define ERROR_CATEGORY(err) (-(err) >> 3)
#define ERROR_INDEX(err) ((-(err) & 7) + 1)

#define CONTROL_ERROR_FIRST_ (-248)
#define CONTROL_ERROR_INHIBITED (CONTROL_ERROR_FIRST_)  /* Movement of the bed has been remotely inhibited. */

#define BED_ERROR_FIRST_ (-240)
#define BED_ERROR_BAD_STATE (BED_ERROR_FIRST_)  /* The bed components are in an unknown or unexpected state. */

#define SPAN_ERROR_FIRST_ (-232)
#define SPAN_ERROR_FAULT (SPAN_ERROR_FIRST_)          /* Motor driver reported a fault. */
#define SPAN_ERROR_DESYNC (SPAN_ERROR_FIRST_ - 1)     /* The actuators lost synchronization. */
#define SPAN_ERROR_DRIVER (SPAN_ERROR_FIRST_ - 2)     /* I/O error communicating with the motor driver. */
#define SPAN_ERROR_TIMEOUT (SPAN_ERROR_FIRST_ - 3)    /* Operation timed out because poll wasn't called often enough. */
#define SPAN_ERROR_NOT_HOME (SPAN_ERROR_FIRST_ - 4)   /* The actuators did not stall at their home position as expected. */
#define SPAN_ERROR_NOT_TRAVEL (SPAN_ERROR_FIRST_ - 5) /* The actuators failed to travel the expected distance without stalling. */

#define LIFT_ERROR_FIRST_ (-224)
#define LIFT_ERROR_FAULT (LIFT_ERROR_FIRST_)       /* Motor driver reported a fault. */
#define LIFT_ERROR_DESYNC (LIFT_ERROR_FIRST_ - 1)  /* The actuators lost synchronization. */
#define LIFT_ERROR_DRIVER (LIFT_ERROR_FIRST_ - 2)  /* I/O error communicating with the motor driver. */
#define LIFT_ERROR_TIMEOUT (LIFT_ERROR_FIRST_ - 3) /* Operation timed out because poll wasn't called often enough. */
#define LIFT_ERROR_STALL (LIFT_ERROR_FIRST_ - 4)   /* Lift stationary when it should be moving or the hall sensors are not working correctly. */
