/* Set of defines related to PID Tuning */
// PID Settings
#ifndef __PID_SETTINGS_H__
#define __PID_SETTINGS_H__

#define GRIP_ON 1.0
#define GRIP_OFF 0.0

enum{SLAVE_1,SLAVE_2};
enum{KP,KI,KD};
enum{GRIP_FACTOR, OPEN_FACTOR, TILT_COMPENSATION};
enum{PITCH, GROSS, YAW1, YAW2, GRIP_SETTINGS};

/*                                                           KP     KI      KD          */
static const double WEAK_END_AFFECTOR_PITCH[3] =            {5.10,  0.00,   0.04};
static const double WEAK_END_AFFECTOR_GROSS[3] =            {2.60,  0.00,   0.02};
static const double WEAK_END_AFFECTOR_YAW1[3]  =            {2.65,  0.00,   0.00};
static const double WEAK_END_AFFECTOR_YAW2[3]  =            {2.65,  0.00,   0.00};
/*                                                          TIGHT   OPEN    TILT PENALTY */
static const double WEAK_END_AFFECTOR_GRIP_SETTINGS[3] =    {1.00, 0.45,   0.00};
static const double* WEAK_END_AFFECTOR[5] = {WEAK_END_AFFECTOR_PITCH, WEAK_END_AFFECTOR_GROSS, WEAK_END_AFFECTOR_YAW1, WEAK_END_AFFECTOR_YAW2, WEAK_END_AFFECTOR_GRIP_SETTINGS};
/*                                                           KP     KI      KD          */
static const double BIG_END_AFFECTOR_PITCH[3] =             {12.0,  0.00,   0.00};
static const double BIG_END_AFFECTOR_GROSS[3] =             {5.00,  0.00,   0.00};
static const double BIG_END_AFFECTOR_YAW1[3]  =             {4.00,  0.00,   0.00};
static const double BIG_END_AFFECTOR_YAW2[3]  =             {4.00,  0.00,   0.00};
/*                                                          TIGHT   OPEN    TILT PENALTY */
static const double BIG_END_AFFECTOR_GRIP_SETTINGS[3] =     {1.00, 0.45,   0.00};
static const double* BIG_END_AFFECTOR[5] = {BIG_END_AFFECTOR_PITCH, BIG_END_AFFECTOR_GROSS, BIG_END_AFFECTOR_YAW1, BIG_END_AFFECTOR_YAW2, BIG_END_AFFECTOR_GRIP_SETTINGS};
/*                                                           KP     KI      KD          */
static const double LOOSE_SUTURE_CUT_PITCH[3] =             {11.00,  0.00,   1.50};
static const double LOOSE_SUTURE_CUT_GROSS[3] =             {20.0,  0.00,   0.65};
static const double LOOSE_SUTURE_CUT_YAW1[3]  =             {19.5,  0.00,   0.55};
static const double LOOSE_SUTURE_CUT_YAW2[3]  =             {19.5,  0.00,   0.55};
/*                                                          TIGHT   OPEN    TILT PENALTY */
static const double LOOSE_SUTURE_CUT_GRIP_SETTINGS[3] =     {1.45,  0.17,   0.02};
static const double* LOOSE_SUTURE_CUT[5] = {LOOSE_SUTURE_CUT_PITCH, LOOSE_SUTURE_CUT_GROSS, LOOSE_SUTURE_CUT_YAW1, LOOSE_SUTURE_CUT_YAW2, LOOSE_SUTURE_CUT_GRIP_SETTINGS};
/*                                                           KP     KI      KD          */
static const double TIGHT_SUTURE_CUT_PITCH[3] =             {8.0,  0.00,   1.50};
static const double TIGHT_SUTURE_CUT_GROSS[3] =             {14.0,  0.00,   0.60};
static const double TIGHT_SUTURE_CUT_YAW1[3]  =             {17.4,  0.00,   0.55}; //back to .04
static const double TIGHT_SUTURE_CUT_YAW2[3]  =             {17.4,  0.00,   0.55}; 
/*                                                          TIGHT   OPEN    TILT PENALTY */
static const double TIGHT_SUTURE_CUT_GRIP_SETTINGS[3] =     {1.75,  0.17,   0.02}; // WAS {1.50, 0.65, 0.00}
static const double* TIGHT_SUTURE_CUT[5] = {TIGHT_SUTURE_CUT_PITCH, TIGHT_SUTURE_CUT_GROSS, TIGHT_SUTURE_CUT_YAW1, TIGHT_SUTURE_CUT_YAW2, TIGHT_SUTURE_CUT_GRIP_SETTINGS};

#define SLAVE_1_END_AFFECTOR    LOOSE_SUTURE_CUT 
#define SLAVE_2_END_AFFECTOR    TIGHT_SUTURE_CUT

#define SLAVE_1_GRIP_FACTOR     SLAVE_1_END_AFFECTOR[GRIP_SETTINGS][GRIP_FACTOR]
#define SLAVE_2_GRIP_FACTOR     SLAVE_2_END_AFFECTOR[GRIP_SETTINGS][GRIP_FACTOR]
#define SLAVE_1_OPEN_FACTOR     SLAVE_1_END_AFFECTOR[GRIP_SETTINGS][OPEN_FACTOR]
#define SLAVE_2_OPEN_FACTOR     SLAVE_2_END_AFFECTOR[GRIP_SETTINGS][OPEN_FACTOR]
#define SLAVE_1_TILT_COMPENSATION     SLAVE_1_END_AFFECTOR[GRIP_SETTINGS][TILT_COMPENSATION]
#define SLAVE_2_TILT_COMPENSATION     SLAVE_2_END_AFFECTOR[GRIP_SETTINGS][TILT_COMPENSATION]

#define PITCH_JOLT false
// Time lag between successive jolts, in microseconds
#define JOLT_PERIOD 10000
#define JOLT_MIN_ERROR 2
#define JOLT_MAX_DERIVATIVE JOLT_MIN_ERROR
#define JOLT_MAX_VARIANCE 4
#define JOLT_VOLTAGE 0.5 * 2

#define WRIST1_P_PITCH      SLAVE_1_END_AFFECTOR[PITCH][KP]
#define WRIST1_I_PITCH      SLAVE_1_END_AFFECTOR[PITCH][KI]
#define WRIST1_D_PITCH      SLAVE_1_END_AFFECTOR[PITCH][KD]

#define WRIST1_P_GROSS      SLAVE_1_END_AFFECTOR[GROSS][KP]
#define WRIST1_I_GROSS      SLAVE_1_END_AFFECTOR[GROSS][KI]
#define WRIST1_D_GROSS      SLAVE_1_END_AFFECTOR[GROSS][KD]

#define WRIST1_P_YAW1       SLAVE_1_END_AFFECTOR[YAW1][KP]
#define WRIST1_I_YAW1       SLAVE_1_END_AFFECTOR[YAW1][KI]
#define WRIST1_D_YAW1       SLAVE_1_END_AFFECTOR[YAW1][KD]

#define WRIST1_P_YAW2       SLAVE_1_END_AFFECTOR[YAW2][KP]
#define WRIST1_I_YAW2       SLAVE_1_END_AFFECTOR[YAW2][KI]
#define WRIST1_D_YAW2       SLAVE_1_END_AFFECTOR[YAW2][KD]

#define WRIST2_P_PITCH      SLAVE_2_END_AFFECTOR[PITCH][KP]
#define WRIST2_I_PITCH      SLAVE_2_END_AFFECTOR[PITCH][KI]
#define WRIST2_D_PITCH      SLAVE_2_END_AFFECTOR[PITCH][KD]

#define WRIST2_P_GROSS      SLAVE_2_END_AFFECTOR[GROSS][KP]
#define WRIST2_I_GROSS      SLAVE_2_END_AFFECTOR[GROSS][KI]
#define WRIST2_D_GROSS      SLAVE_2_END_AFFECTOR[GROSS][KD]

#define WRIST2_P_YAW1       SLAVE_2_END_AFFECTOR[YAW1][KP]
#define WRIST2_I_YAW1       SLAVE_2_END_AFFECTOR[YAW1][KI]
#define WRIST2_D_YAW1       SLAVE_2_END_AFFECTOR[YAW1][KD]

#define WRIST2_P_YAW2       SLAVE_2_END_AFFECTOR[YAW2][KP]
#define WRIST2_I_YAW2       SLAVE_2_END_AFFECTOR[YAW2][KI]
#define WRIST2_D_YAW2       SLAVE_2_END_AFFECTOR[YAW2][KD]



#define BASE_1_PID_REDUCTION 0.90 // 0.90 for servo
#define BASE_2_PID_REDUCTION 0.90 //0.85 for servo
//#define BASE_PID_REDUCTION 0.0


#define BASE1_LEFT_P    83.0 * BASE_1_PID_REDUCTION // WAS 83, 2.0
#define BASE1_LEFT_I    0 * BASE_1_PID_REDUCTION
#define BASE1_LEFT_D    5.0 * BASE_1_PID_REDUCTION

#define BASE1_RIGHT_P   83.0 * BASE_1_PID_REDUCTION
#define BASE1_RIGHT_I   0 * BASE_1_PID_REDUCTION
#define BASE1_RIGHT_D   5.0 * BASE_1_PID_REDUCTION

#define BASE1_REAR_P    88.0 * BASE_1_PID_REDUCTION //WAS 83 * BASE_PID_REDUCTION
#define BASE1_REAR_I    0 * BASE_1_PID_REDUCTION
#define BASE1_REAR_D    5.0 * BASE_1_PID_REDUCTION



#define BASE2_LEFT_P    83.0 * BASE_2_PID_REDUCTION
#define BASE2_LEFT_I    0 * BASE_2_PID_REDUCTION
#define BASE2_LEFT_D    5.0 * BASE_2_PID_REDUCTION

#define BASE2_RIGHT_P   83.0 * BASE_2_PID_REDUCTION
#define BASE2_RIGHT_I   0 * BASE_2_PID_REDUCTION
#define BASE2_RIGHT_D   5.0 * BASE_2_PID_REDUCTION

#define BASE2_REAR_P    85.0 * BASE_2_PID_REDUCTION
#define BASE2_REAR_I    0 * BASE_2_PID_REDUCTION
#define BASE2_REAR_D    5.0 * BASE_2_PID_REDUCTION

#endif
