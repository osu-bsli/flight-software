#ifndef FC_ARMING_H
#define FC_ARMING_H

void fc_arming_arm_accelerometer_1(void);
void fc_arming_disarm_accelerometer_1(void);
int fc_arming_is_accelerometer_1_armed(void);

void fc_arming_arm_accelerometer_2(void);
void fc_arming_disarm_accelerometer_2(void);
int fc_arming_is_accelerometer_2_armed(void);

void fc_arming_arm_barometer(void);
void fc_arming_disarm_barometer(void);
int fc_arming_is_barometer_armed(void);

void fc_arming_arm_magnetometer(void);
void fc_arming_disarm_magnetometer(void);
int fc_arming_is_magnetometer_armed(void);

#endif
