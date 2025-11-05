#ifndef UTM_H
#define UTM_H
#include <stdint.h>

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
    double easting;
    double northing;
} UTMCoord;

UTMCoord latlon_to_utm(double lat, double lon);

#endif // UTM_H