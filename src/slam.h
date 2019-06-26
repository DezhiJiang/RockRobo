//
// Created by nyx on 18-6-25.
//

#pragma once

#include "slam_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	int trajectory_id ;          //轨迹id

	//对应sensor_id是其topic的名字
	std::string sensor_id_laser ;
	std::string sensor_id_multi_laser ;
	std::string sensor_id_imu ;
	std::string sensor_id_odo ;

}rock_slam,  *rock_slam_t;



enum RockSlamSystem {
	ROCK_SLAM_SYSTEM_LASER  = 0,
	ROCK_SLAM_SYSTEM_VISION = 1
};

typedef int32_t rock_result_t;

rock_slam_t rock_slam_create(RockSlamSystem system);

void rock_slam_release(rock_slam_t* slam);

rock_result_t rock_slam_get_pose(rock_slam_t slam, rock_pose_2d_t* pose);

rock_result_t rock_slam_set_pose(rock_slam_t slam, rock_pose_2d_t const* pose);

rock_result_t rock_slam_lock(rock_slam_t slam, bool lock);

rock_result_t rock_slam_locked(rock_slam_t slam, bool* locked);

rock_result_t rock_slam_pause(rock_slam_t slam, bool pause);

rock_result_t rock_slam_paused(rock_slam_t slam, bool* paused);

rock_result_t rock_slam_move(rock_slam_t slam, rock_motion_t const* motion);

rock_result_t rock_slam_observe(rock_slam_t slam, rock_observation_t const* observation);

rock_result_t rock_slam_reset(rock_slam_t slam);

rock_result_t rock_slam_map_used(rock_slam_t slam, rock_rect_t* rect);

rock_result_t rock_slam_map(rock_slam_t slam, void* buffer, rock_size_t size);

rock_result_t rock_slam_map_update(rock_slam_t slam, rock_pose_2d_t* pose);

rock_result_t rock_slam_relocate_prepare(rock_slam_t slam, bool start);

rock_result_t rock_slam_relocate(rock_slam_t slam, rock_bool_t const* cancel);

rock_result_t rock_slam_range_relocate(rock_slam_t slam, rock_pose_2d_t pose, rock_pose_2d_t range);

rock_result_t rock_slam_save_map(rock_slam_t slam, char const* path);

rock_result_t rock_slam_load_map(rock_slam_t slam, char const* path);

rock_result_t rock_slam_delete_map(rock_slam_t slam, char const* path);

rock_result_t rock_slam_rotate_map(rock_slam_t slam, rock_pose_2d_t const* pose);

void rock_slam_set_logger(rock_logger_t logger);

void rock_slam_set_log_level(uint32_t level);

void* rock_slam_memory_allocate(rock_size_t size);

void rock_slam_memory_release(void* ptr);

#ifdef __cplusplus
};
#endif
