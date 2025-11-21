#ifndef UTM_H
#define UTM_H

#define _USE_MATH_DEFINES
#include <math.h>

#define DEG_TO_RAD (M_PI / 180.0)

// WGS84 ellipsoid parameters
#define WGS84_A 6378137.0           // Semi-major axis (meters)
#define WGS84_E 0.0818191908426     // First eccentricity
#define WGS84_E2 0.00669437999014   // e squared

// UTM scale factor
#define UTM_K0 0.9996

typedef struct {
    float easting;
    float northing;
} utm_coord_t;

utm_coord_t latlon_to_utm(float lat, float lon);

#endif // UTM_H