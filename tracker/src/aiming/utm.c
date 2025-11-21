#include "utm.h"
#include <math.h>

utm_coord_t latlon_to_utm(float lat, float lon) {
    utm_coord_t utm;
    
    // Calculate zone for central meridian
    int zone = (int)((lon + 180.0) / 6.0) + 1;
    
    // Convert to radians
    float lat_rad = lat * DEG_TO_RAD;
    float lon_rad = lon * DEG_TO_RAD;
    
    // Central meridian for the zone
    float lon0 = ((zone - 1) * 6.0 - 180.0 + 3.0) * DEG_TO_RAD;
    float lon_diff = lon_rad - lon0;
    
    // Pre-calculate trig functions
    float sin_lat = sin(lat_rad);
    float cos_lat = cos(lat_rad);
    float tan_lat = tan(lat_rad);
    
    // Calculate N (radius of curvature in prime vertical)
    float N = WGS84_A / sqrt(1 - WGS84_E2 * sin_lat * sin_lat);
    
    // Calculate T and C
    float T = tan_lat * tan_lat;
    float C = WGS84_E2 * cos_lat * cos_lat / (1 - WGS84_E2);
    float A = cos_lat * lon_diff;
    
    // Calculate M (meridional arc)
    float e4 = WGS84_E2 * WGS84_E2;
    float e6 = e4 * WGS84_E2;
    
    float M = WGS84_A * ((1 - WGS84_E2/4 - 3*e4/64 - 5*e6/256) * lat_rad
                - (3*WGS84_E2/8 + 3*e4/32 + 45*e6/1024) * sin(2*lat_rad)
                + (15*e4/256 + 45*e6/1024) * sin(4*lat_rad)
                - (35*e6/3072) * sin(6*lat_rad));
    
    // Calculate UTM easting
    utm.easting = UTM_K0 * N * (A + (1-T+C)*A*A*A/6
                  + (5-18*T+T*T+72*C-58*WGS84_E2)*A*A*A*A*A/120)
                  + 500000.0; // False easting
    
    // Calculate UTM northing
    utm.northing = UTM_K0 * (M + N*tan_lat * (A*A/2
                   + (5-T+9*C+4*C*C)*A*A*A*A/24
                   + (61-58*T+T*T+600*C-330*WGS84_E2)*A*A*A*A*A*A/720));
    
    // Add false northing for southern hemisphere
    if (lat < 0) {
        utm.northing += 10000000.0;
    }
    
    return utm;
}