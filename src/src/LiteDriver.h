//
// Created by leon on 4/8/18.
//

#ifndef LITEDRIVER_LITEDRIVER_H
#define LITEDRIVER_LITEDRIVER_H

#include <libplayercore/playercore.h>
#include <map>
#include <vector>
#include <utility>
#include <pthread.h>
#include "SensorData.h"
#include <atomic>

//以下头文件是属于bridge部分的代码
#include "start.h"
#include "start_options.h"
#include "cartographer/mapping/map_builder.h"

//以下头文件用于保存地图
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "slam.h"
#include "slam_types.h"
#include "slam_error.h"

#include "cartographer/io/proto_stream.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/proto/submap.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/submap_2d.h"

//using namespace std;

static void* UpdateSpeedThread(void* obj);


class LiteDriver : public ThreadedDriver {
public:
    typedef void (LiteDriver::*ProcessFun)(double, void*);
    //typedef void (LiteDriver::*ProcessFun)(void*);

    LiteDriver(ConfigFile* cf, int section);
    virtual ~LiteDriver();
    int MainSetup();
    void MainQuit();
    int ProcessMessage(QueuePointer &resp_queue, player_msghdr * hdr, void * data);
    void SetSpeed(double lineSpeed, double yawSpeed, bool force = false);
    void UpdateSpeed();
public:
    //Update wheel speed thread
    pthread_t m_update_speed_thread;

    int listenfd;
    int connectfd;

    //用于保存地图
    typedef struct {
        rock_rect_t region;  //地图区域,包括起始点和大小
        rock_occupied_value_t occupied_value[1024][1024];
        //横向x轴，纵向y轴
        //(-512,-512)               (511,-512)     | (0,0)                           (1023,0)
        //                ...                      |
        //               (0,-1)                    |                (512,511)
        //    ... (-1,0) (0,0) (1,0) ...           |      (511,512) (512,512) (513,512)
        //               (0,1)                     |                (512,513)
        //                ...                      |
        //(-512,511)                (511,511)      | (0,1023)                        (1023,1023)
        //左：地图坐标，右：数组坐标
        //左+(512,512)=右
        //右+(-512,-512)=左

    }GridMap;   //定义一个结构体，没有typedef则是定义的同时进行声明
    GridMap grid_map;

    //地图类别
    enum MapType{
        ALL = 1,
        USED,
        UPDATE
    };
private:
    virtual void Main();
    bool SubscribeDevice(player_devaddr_t& addr,
                         ConfigFile* cf, int section,
                         int32_t interf_code,
                         int32_t index,
                         const char *key,
                         Device** ppDevice);
    bool RegisterProcessFun(int type,
                            int subtype,
                            player_devaddr_t addr,
                            void (LiteDriver::*ProcessFun)(double, void*));
    //void (LiteDriver::*ProcessFun)(void*));
    bool AddInterface(player_devaddr_t& addr,
                      ConfigFile* cf,
                      int section,
                      int32_t interf_code,
                      int32_t index,
                      const char *key);
    void ProcessBumperData(double time, void* pData);
    void ProcessKeyData(double time, void* pData);
    void ProcessLaserData(double time, void* pData);
    void ProcessGyroAndOdoData(double time, void* pData);
    //Test For New Odom Data
    void ProcessNewOdoData(double time, void* pData);
    //void ProcessBumperData(void* pData);
    //void ProcessKeyData(void* pData);
    //void ProcessLaserData(void* pData);
    //void ProcessGyroAndOdoData(void* pData);

    void SendSpeedCMD(double lineSpeed, double yawSpeed, bool force = false);
    void BumperSet();

    ///设置定时器
//    void SetTimer(double time, struct pollfd *fds);
    void SetTimer(struct pollfd *fds);
    //设置普通定时器，用于解决碰撞问题
    void InitTimer(struct pollfd *fds);
    //这个定时器只触发一次
    void SetOnceTimer(struct pollfd *fds, int first_time);

private:
    player_devaddr_t m_bumper_addr;
    player_devaddr_t m_odo_addr;
    player_devaddr_t m_key_addr;
    player_devaddr_t m_laser_addr;
    player_devaddr_t m_gyro_odo_addr;
    player_devaddr_t m_controlAio_addr;
    //Test For New Odom Data
    player_devaddr_t m_new_odo_addr;

    bool m_stop_driver;
    Device* m_pWheelDevice;

    std::atomic<double> m_line_speed;  // m/s
    std::atomic<double> m_yaw_speed;   // rad
    std::atomic<bool> m_force;        // force set, ignore cliff

    std::map<std::pair<int, int>, std::vector<std::pair<player_devaddr_t, ProcessFun>>> m_register_table;


public:
    void SaveMap(MapType maptype, char const* path);   //保存地图的接口

    //新添加的接口
    ::cartographer::transform::Rigid3d getCurGlobalPose(const int trajectory_id);  //获取全局位姿的接口
    ::cartographer::transform::Rigid3d getCurLocalPose();   //获取局部位姿的接口

    //用于测试接口用到的结构体变量
    rock_slam_t test_slam;     //slam句柄变量。用于测试接口

    //北京公司要求接口
    rock_slam_t _rock_slam_create(RockSlamSystem system);   //创建slam句柄
    void _rock_slam_release(rock_slam_t* slam);             //释放slam句柄

    void _rock_slam_move(rock_slam_t slam,rock_motion_t const* motion);              //move方法向slam传入odo和gyro数据
    void _rock_slam_observe(rock_slam_t slam,rock_observation_t const* observation); //observe方法向slam传入laser数据

    void _rock_slam_map_used(rock_slam_t slam, rock_rect_t* rect); //获取当前地图上已使用的区域
    void _rock_slam_map(rock_slam_t slam, rock_occupied_value_t* buffer, rock_size_t size); //获取当前完整三值（unknown，free，obstacle）地图的一维数组
    void _rock_slam_map_update(rock_slam_t slam, rock_pose_2d_t* pose); //返回单次更新的区域

    rock_rect_t* rect_lock;                                      //锁住地图时的已使用地图区域
    rock_occupied_value_t* gridmap_lock;              //锁住地图时的地图
    bool lockflag = false;                                       //锁地图标志位
    void _rock_slam_lock(rock_slam_t slam, bool lock);           //锁地图
    void _rock_slam_locked(rock_slam_t slam, bool* locked);      //查询地图是否已锁

    rock_logger_t mylogfunc; //日志函数
    int32_t log_level;       //日志等级
    void _rock_slam_set_logger(rock_logger_t logger);            //设置日志函数
    void _rock_slam_set_log_level(uint32_t level);                //设置日志级别


private:
    //此方法用于处理submap,使之成为可以拼接的状态
    void FillSubmapSlice(
            const ::cartographer::transform::Rigid3d& global_submap_pose,
            const ::cartographer::mapping::proto::Submap& proto,
            ::cartographer::io::SubmapSlice* submap_slice);

};

#endif //LITEDRIVER_LITEDRIVER_H
