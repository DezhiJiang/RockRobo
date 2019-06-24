//
// Created by leon on 4/10/18.
//

#ifndef LITEDRIVER_SENSORDATA_H
#define LITEDRIVER_SENSORDATA_H

/*key*/
typedef enum {
    KEY_NONE                                           = 0,
    START_STOP_SHORT                                   = 1,
    START_STOP_LONG                                    = 2,
    DOCK_BACK_SHORT                                    = 3,
    DOCK_BACK_LONG                                     = 4,
    COMPOSITE_LONG                                     = 6,
    KEY_RPT_DOWN                                       = 8,

    KEY_RPT_SPOT_SHORT                                 = 11,
    KEY_RPT_SPOT_LONG                                  = 12,
    KEY_RPT_HOME_LONG_10                               = 14,
    KEY_INVALID                                        = 0xff,
} KEY_VALUE;

#define WHEEL_SPEED_UPDATE_INTERVAL 50 //ms
#define Laser_Intensity_Mask 0xFFF

static const uint16_t LDS_FLAG_DATA_INVALID = 0x8000;
static const uint16_t LDS_STRENGTH_WARNING = 0x4000;
static const uint16_t LDS_FLAG_COVER_FILTER = 0x2000;
static const uint16_t LDS_FLAG_SPEED_WARNING = 0x1000;

#endif //LITEDRIVER_SENSORDATA_H
