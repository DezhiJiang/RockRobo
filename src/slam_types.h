//
// Created by nyx on 18-6-25.
//

#pragma once

#include <stdint.h>
#include <float.h>

#ifndef __cplusplus

#include <stdbool.h>

#endif

typedef uint32_t      rock_size_t;
typedef int32_t       rock_int_t;
typedef float         rock_real_t;
typedef volatile bool rock_bool_t;

typedef rock_real_t rock_radian_t;
typedef rock_real_t rock_timestamp_t;
typedef rock_real_t rock_weight_t;
typedef rock_real_t rock_scale_t;

#define ROCK_METRE_EPSILON      FLT_EPSILON
#define ROCK_RADIAN_EPSILON     FLT_EPSILON
#define ROCK_WEIGHT_EPSILON     FLT_EPSILON
#define ROCK_REAL_MAX           FLT_MAX
#define ROCK_LASER_COUNT_MAX    360

typedef float   rock_metre_t;
typedef int32_t rock_grid_t;
typedef uint8_t rock_occupied_value_t;
typedef uint8_t rock_counting_value_t;
typedef uint8_t rock_pixel_t;

typedef rock_metre_t  rock_metre_per_sec_t;
typedef rock_radian_t rock_radian_per_sec_t;
typedef rock_metre_t  rock_metre_per_sec2_t;

typedef int (* rock_logger_t)(int, int, int, char const*, char const*, ...);

typedef struct {
	union {
		rock_grid_t  grid;
		rock_metre_t metre;
	};

} rock_coord_t;

typedef struct {
	rock_radian_t roll, pitch, yaw;
} rock_euler_t;

typedef struct {
	int16_t num;
	int16_t den;
} rock_ratio_t;

typedef struct {
	rock_occupied_value_t value;
} rock_occupied_cell_t;

typedef struct {
	uint8_t value;
} rock_byte_t;

typedef struct {
	rock_size_t size;
	uint8_t     data[0];
} rock_buffer_t;

typedef struct {
	rock_coord_t x, y;
} rock_point_2d_t;

typedef struct {
	rock_coord_t x, y, z;
} rock_point_3d_t;

typedef struct {
	rock_point_2d_t point;
	rock_radian_t   theta;
} rock_pose_2d_t;

typedef struct {
	rock_point_3d_t point;
	rock_euler_t    euler;
} rock_pose_3d_t;

typedef struct {
	rock_metre_t  trans;
	rock_radian_t rot1;
	rock_radian_t rot2;
} rock_motion_decomposition_t;

typedef struct {
	rock_size_t width, height;
} rock_size_2d_t;

typedef struct {
	rock_point_2d_t point;
	rock_size_2d_t  size;
} rock_rect_t;

typedef struct {
	rock_metre_t  rho;
	rock_radian_t theta;
} rock_polar_t;

typedef struct {
	rock_polar_t polar;
	uint16_t     intensity;
	uint16_t     flag;
} rock_beam_t;

// The sizeof of rock_scan_t and rock_image_t should be the same.
typedef struct {
	uint32_t    id;
	rock_size_t count;
	rock_beam_t beams[0];
} rock_scan_t;

typedef struct {
	uint32_t    type;
	rock_size_t size;
	uint8_t     data[0];
} rock_image_t;    //for VSLAM?

typedef struct {
	rock_int_t left, right;
} rock_wheel_t;

typedef struct {
	rock_metre_per_sec2_t x, y, z;
} rock_acc_t;

typedef struct {
	rock_radian_per_sec_t roll, pitch, yaw;
} rock_angular_vel_3d_t;

typedef struct {
	rock_euler_t          euler;
	rock_angular_vel_3d_t vel;
} rock_gyro_t;

typedef struct {
	rock_metre_per_sec_t  v;
	rock_radian_per_sec_t w;
} rock_velocity_t;


// ********** Sensor Data Type **********
typedef struct {
	rock_timestamp_t timestamp;
	rock_pose_2d_t   pose;
	rock_wheel_t     wheel;
} rock_odometry_t;

typedef struct {
	rock_timestamp_t timestamp;
	rock_acc_t       acc;
	rock_gyro_t      gyro;
} rock_imu_t;

typedef struct {
	rock_timestamp_t timestamp;
	rock_velocity_t  velocity;
} rock_command_t;

typedef struct {
	rock_timestamp_t timestamp;
	rock_pose_2d_t   pose;
	rock_wheel_t     wheel;
	rock_acc_t       acc;
	rock_gyro_t      gyro;
	rock_velocity_t  velocity;
} rock_motion_t;

typedef struct {
	rock_timestamp_t timestamp;
	bool             left, front, right;
} rock_bumper_t;

typedef struct {
	rock_timestamp_t timestamp;
	union {
		rock_scan_t  scan;
		rock_image_t image;
	};
} rock_observation_t;


// ********** Occupied Map Cell Value **********
enum {
	ROCK_OCCUPIED_UNKNOWN  = 0,
	ROCK_OCCUPIED_FREE     = 255,
	ROCK_OCCUPIED_OBSTACLE = 1
};

enum {
	ROCK_IMAGE_TYPE_IGN = 0,
	ROCK_IMAGE_TYPE_YUV = 1,
	ROCK_IMAGE_TYPE_BMP = 2,
	ROCK_IMAGE_TYPE_RAW = 3,
	ROCK_IMAGE_TYPE_PGM = 4
};
