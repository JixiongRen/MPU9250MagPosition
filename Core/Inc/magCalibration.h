#ifndef MAG_CALIBRATION_H
#define MAG_CALIBRATION_H

#include <stdint.h>

extern const float magHardIron[12][3];
extern const float magSoftIron[12][3][3];

static inline void applyMagCalibration(uint8_t sensor_id, const float *raw_mag, float *calibrated_mag) {
    float mag_centered[3];
    for (int i = 0; i < 3; i++) {
        mag_centered[i] = raw_mag[i] - magHardIron[sensor_id][i];
    }
    for (int i = 0; i < 3; i++) {
        calibrated_mag[i] = 0.0f;
        for (int j = 0; j < 3; j++) {
            calibrated_mag[i] += magSoftIron[sensor_id][i][j] * mag_centered[j];
        }
    }
}

#endif
