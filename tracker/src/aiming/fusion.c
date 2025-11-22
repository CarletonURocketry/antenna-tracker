#include <uORB/uORB.h>
#include "aiming.h"
#include <math.h>

/* The pressure at sea level in millibar*/
#define SEA_PRESSURE (1013.25)

/* The universal gas constant */
#define GAS_CONSTANT (8.31432)

/* Constant for acceleration due to gravity */
#define GRAVITY (9.80665)

/* Constant for the mean molar mass of atmospheric gases */
#define MOLAR_MASS (0.0289644)

/* Constant for the conversion from Celsius to Kelvin */
#define KELVIN (273)


ORB_DECLARE(sensor_baro);

/* Definition for fusion uORB dopic */
#if defined(CONFIG_DEBUG_UORB)
static const char sensor_alt_format[] = "altitude - timestamp:%" PRIu64 ",altitude:%hf";
ORB_DEFINE(sensor_alt, struct sensor_alt, sensor_alt_format);
#else
ORB_DEFINE(sensor_alt, struct sensor_alt, 0);
#endif


int calculate_altitude(struct sensor_baro *baro_data, struct sensor_alt *altitude) {
    altitude->timestamp = baro_data->timestamp;
    altitude->altitude = -(GAS_CONSTANT * (KELVIN + baro_data->temperature)) / (MOLAR_MASS * GRAVITY) *
                    log(baro_data->pressure / SEA_PRESSURE);
    
    return 0;
};