#ifndef OPERATING_LIMITS_H
#define OPERATING_LIMITS_H

// ------------------------------
// Operating Limits
// ------------------------------

#define MAX_DURATION 300 * 60 * 1000 // 300 minutes or 5 hours
#define MIN_TEMP 0
#define MAX_TEMP 400

// Output limit
#define MAX_OUTPUT 255 * 0.75 // Maximum output for PID; this is set to 80% of the maximum ON time
// change this when you wish to limit the output to a certain range this may be required for certain
// heating elements that are prone to overheating (channel strip heaters, etc.)

// PID constant limits
#define MIN_KP 0
#define MAX_KP 2000
#define MIN_KI 0
#define MAX_KI 500
#define MIN_KD 0
#define MAX_KD 100

// Adaptive PID constant limits
#define MIN_CP 0
#define MAX_CP 500
#define MIN_CI 0
#define MAX_CI 10
#define MIN_CD 0
#define MAX_CD 1

#define MIN_SET_DURATION 1000
#define MAX_SET_DURATION 300 * 60 * 1000 // 300 minutes or 5 hours

#endif // OPERATING_LIMITS_H
