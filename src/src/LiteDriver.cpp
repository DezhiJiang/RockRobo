//
// Created by leon on 4/8/18.
//

#include "LiteDriver.h"
#include "utility.h"
#include "slam_types.h"
#include "slam.h"
#include <thread>

#include <sys/poll.h>
#include <sys/time.h>
#include <sys/timerfd.h>
#include <unistd.h>
#include <tuple>

#include <ctime>

#include <iostream>
#include <iomanip>

///用于控制连接
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

//using namespace std;
//声明命名空间
using namespace RockRobo;

//目前将启动变量(start)写成一个全局变量，这样相对简单
namespace {
    StartOptions start_options;
    TrajectoryOptions trajectory_options;

    constexpr int trajectory_id = 0;
    //对应sensor_id是其topic的名字
    const std::string sensor_id_laser = "scan";
    const std::string sensor_id_multi_laser = "echoes";
    const std::string sensor_id_imu = "imu";
    const std::string sensor_id_odo = "odom";

//TODO:定义读取lua配置文件的目录
    const std::string configuration_directory = "configuration_files";
    const std::string configuration_basename = "backpack_2d.lua";

    ///序列化状态的文件名
    const std::string save_state_filename = "trajectory_state.pbstream";

    //FIXME:这个首次超时时间大小需要测试
    constexpr int FIRST_TIME = 10;
    //触发poll的时间
    int TRI_TIME = 20;

    //预热等待laser的标志位
    int LASER_START = 0;

    ///控制命令常量
    constexpr int PORT = 1234;
    constexpr int BACKLOG = 1;  //constexpr编译期
    constexpr int MAXDATASIZE = 10;

    //表示碰撞需要停止odo数据的传输
    bool g_stop_odo = false;
    //用于标示第一次转向是否进行
    bool g_skip = false;
    //表示调整转向需要的时间
    constexpr int kFirstBumperTime = 9;
    //表示后退时间
    constexpr int kRunTime = 100000000;

    //设置LocalSlam回调函数定时器,之前这部分是局部变量，因为需要配合碰撞处理，将其修改为全局变量
    struct pollfd fds[3];
    constexpr int kFdNum = 3;
    //碰撞处理状态保存量
            // 0:表示第一次调整方向
            // 1:表示前进
            // 2:表示第二次调整方向 现在不调整第二次方向
    int state_num = 0;
    enum{
        TURN,
        STRAIGHT
    };

    //用于保存地图


}

//将start改为指针，将其放在MainSetUp函数中进行初始化
Start *start;

LiteDriver::LiteDriver(ConfigFile* cf, int section):
        ThreadedDriver(cf, section, true, PLAYER_MSGQUEUE_DEFAULT_MAXLEN),
        m_stop_driver(false),
        m_pWheelDevice(nullptr),
        m_line_speed(0.0),
        m_yaw_speed(0.0),
        m_force(false)
{
    //
    SubscribeDevice(m_bumper_addr, cf, section, PLAYER_BUMPER_CODE, -1, "base_bumper", nullptr);
    SubscribeDevice(m_key_addr, cf, section, PLAYER_KEYPAD_CODE, -1, nullptr, nullptr);
    SubscribeDevice(m_laser_addr, cf, section, PLAYER_LASER_CODE, -1, "slam", nullptr);
    SubscribeDevice(m_gyro_odo_addr, cf, section, PLAYER_POSITION3D_CODE, -1, "odo_gyro_for_slam", nullptr);
    //Test For New Odom Data
    SubscribeDevice(m_new_odo_addr, cf, section, PLAYER_POSITION3D_CODE, -1, "odo", nullptr);
    ///目前的数据都是按照3d完成的
    SubscribeDevice(m_odo_addr, cf, section, PLAYER_POSITION2D_CODE, -1, "odo2d", &m_pWheelDevice);
    AddInterface(m_controlAio_addr, cf, section, PLAYER_AIO_CODE, -1, "control");
    Litelog(LEVEL_INFO, "Init Lite Driver finish.\n");
}

LiteDriver::~LiteDriver(){}

int LiteDriver::MainSetup(){
    fds[0].fd = -1;
    fds[1].fd = -1;
    fds[2].fd = -1;
    Litelog(LEVEL_INFO, "Lite Driver Main Setup.\n");
    RegisterProcessFun(PLAYER_MSGTYPE_DATA, PLAYER_BUMPER_DATA_STATE, m_bumper_addr, &LiteDriver::ProcessBumperData);
    RegisterProcessFun(PLAYER_MSGTYPE_DATA, PLAYER_KEYPAD_DATA_STATE, m_key_addr, &LiteDriver::ProcessKeyData);
//	RegisterProcessFun(PLAYER_MSGTYPE_DATA, PLAYER_LASER_DATA_SCANANGLE, m_laser_addr, &LiteDriver::ProcessLaserData);
//	RegisterProcessFun(PLAYER_MSGTYPE_DATA, PLAYER_POSITION3D_DATA_STATE, m_gyro_odo_addr, &LiteDriver::ProcessGyroAndOdoData);
    Litelog(LEVEL_INFO, "Lite Driver Main Setup finish.\n");
//    char buf[255];
//    if(getcwd(buf, 250))
//        Litelog(LEVEL_INFO, "current path:%s.\n", buf);

//    std::tie(start_options, trajectory_options) =
//            LoadOptions(configuration_directory, configuration_basename);
    LoadOptions(configuration_directory, configuration_basename, start_options, trajectory_options);
    auto map_builder =
            cartographer::common::make_unique<cartographer::mapping::MapBuilder>(
                    start_options.map_builder_options);
    start = new Start(start_options, std::move(map_builder));

//    LOG(INFO)<<"LoadState start.\n";
//    if(!save_state_filename.empty())
//        start->LoadState(save_state_filename,false);
//    Litelog(LEVEL_INFO, "LoadState finish.\n");
//    LOG(INFO)<<"LoadState finish.\n";

//    start->getNewTrajectoryID(save_state_filename);

//    start->FinishAllTrajectory();
//    start->RunFinalOptimization();

//    Litelog(LEVEL_INFO, "SerializeState start.\n");
//    LOG(INFO)<<"SerializeState start.\n";
//    start->SerializeState("jdz.pbstream");
//    LOG(INFO)<<"SerializeState finish.\n";
//    Litelog(LEVEL_INFO, "SerializeState finish.\n");

    //LOG(INFO)<<"------------------------MainSet---------------------------\n";

    return 0;
}

void LiteDriver::MainQuit(){
    start->FinishAllTrajectory();
    start->RunFinalOptimization();
    start->SerializeState(save_state_filename);

    //SaveMap(ALL, "test");

    //LOG(INFO)<<"------------------------MainQuit---------------------------\n";

    //------------------------Test Begin----------------------------
    //SaveMap(ALL, "test");


    //句柄创建和已使用地图接口测试
    test_slam = _rock_slam_create(ROCK_SLAM_SYSTEM_LASER); //创建句柄
    rock_rect_t* test_rect =(rock_rect_t*)malloc(sizeof(rock_rect_t));
    _rock_slam_map_used(test_slam,test_rect);

    //获取地图接口测试
    uint32_t  size = 1024*1024*sizeof(rock_occupied_value_t) ;
//    rock_occupied_value_t* buffer = (rock_occupied_value_t*)malloc(size);
    rock_occupied_value_t* buffer = new rock_occupied_value_t[size];
    _rock_slam_map(test_slam, buffer, size);

    //锁地图接口测试
    rect_lock = (rock_rect_t*)malloc(sizeof(rock_rect_t));
    gridmap_lock = new rock_occupied_value_t[1024*1024];

    _rock_slam_lock(test_slam,true);


    //------------------------Test End----------------------------

    //关闭定时器，防止内存泄露
    if(fds[0].fd != -1){
        close(fds[0].fd);
    }
    if(fds[2].fd != -1){
        close(fds[2].fd);
    }
    close(connectfd);
    close(listenfd);

    delete start;
    Litelog(LEVEL_INFO, "Lite Driver Main Quit.\n");
}

int LiteDriver::ProcessMessage(QueuePointer &resp_queue, player_msghdr * hdr, void * data){
    int ret = -1;
    if(m_register_table.find(std::make_pair(hdr->type, hdr->subtype)) != m_register_table.end()){
        for(auto iter: m_register_table[std::make_pair(hdr->type, hdr->subtype)]){
            if(Message::MatchMessage(hdr, hdr->type, hdr->subtype, iter.first))
            {
                double time = hdr->timestamp;
                (this->*iter.second)(time, data);
                //(this->*iter.second)(data);
                ret = 0;
            }
        }
    }
    return ret;
}

bool LiteDriver::RegisterProcessFun(int type,
                                    int subtype,
                                    player_devaddr_t addr,
                                    void (LiteDriver::*ProcessFun)(double, void*)){
    //void (LiteDriver::*ProcessFun)(void*)){
    m_register_table[std::make_pair(type, subtype)].emplace_back(std::make_pair(addr, ProcessFun));
    return true;
}

void LiteDriver::ProcessBumperData(double time_p, void* pData){
    //void LiteDriver::ProcessBumperData(void* pData){
    player_bumper_data_t bumperData = *static_cast<player_bumper_data_t *>((void*) pData);
    Litelog(LEVEL_INFO, "Bumper data [left/mid/right] : [%d, %d, %d].\n",
            (int)bumperData.bumpers[0],
            (int)bumperData.bumpers[1],
            (int)bumperData.bumpers[2]);

//    static time_t pre = time(NULL);
//    static time_t now = 0;
//    now = time(NULL);


//    static int first_in = 0;

    //TODO:这个时间间隔1设置是否合理需要测试
    // >1 防止第一次设置时进入， 第二个判断条件则是防止在过滤数据中途设置速度
//    if(now-pre > 1 && !g_stop_odo){
//        SetSpeed(0, 0);
//        pre = now;
//    }

//    if(first_in++)
//        g_stop_odo = true;

    if(((int)bumperData.bumpers[0] || (int)bumperData.bumpers[1] || (int)bumperData.bumpers[2])
       && !g_stop_odo){
        SetSpeed(0, 0);
        g_stop_odo = true;
    }

}

void LiteDriver::ProcessKeyData(double time, void* pData){
    //void LiteDriver::ProcessKeyData(void* pData){
    player_keypad_data_t keyData = *static_cast<player_keypad_data_t*>(pData);
    Litelog(LEVEL_INFO, "Key data: %d.\n", (int)keyData.status);
    switch(keyData.status){
        case DOCK_BACK_SHORT:
            SetSpeed(0.0, M_PI/16);
            break;
        case KEY_RPT_SPOT_SHORT:
            SetSpeed(0.2, 0.0);
            break;
        case START_STOP_SHORT:
            ++LASER_START;
            break;
        default:
            break;
    }
}

void LiteDriver::ProcessLaserData(double time, void* pData){
    //void LiteDriver::ProcessLaserData(void* pData){
    static int printl = 0;
    if(printl++ % 10 == 0) {
        Litelog(LEVEL_INFO, "Laser data.\n");
    }
    //此处注册laser数据的回调,其中两个参数固定,
    //TODO::需要验证是哪个laser
    start->HandleLaserScanMessage(trajectory_id, sensor_id_laser, time, pData);
//    start.HandleMultiEchoScanMessage(trajectory_id, sensor_id_multi_laser, time, pData);
    //    player_laser_data_scanangle_t laserData = *static_cast<player_laser_data_scanangle_t *>(pData);
    //    for(int i = 0; i< laserData.ranges_count; i++){
    //        Litelog(LEVEL_INFO, "%f\t: %f\t %d\t.  Invalid: %x\n",
    //                laserData.angles[i],
    //                laserData.ranges[i],
    //                ((uint16_t)laserData.intensity[i])&Laser_Intensity_Mask,
    //                (((uint16_t)laserData.intensity[i])&0xF000) != 0x0000);
    //    }
}

void LiteDriver::BumperSet() {

    //循环使用第三个描述符
    //关闭定时器，防止内存泄露
    SetSpeed(0.0, M_PI/9);
//    if(fds[2].fd != -1){
//        close(fds[2].fd);
//    }
    //timerfd 只创建一次
    if(fds[2].fd == -1){
        InitTimer(&fds[2]);
    }
    SetOnceTimer(&fds[2], TURN);
    state_num = 0;
}

void LiteDriver::InitTimer(struct pollfd *fds){
    int tfd = timerfd_create(CLOCK_REALTIME, TFD_NONBLOCK);
    if(tfd < 0)
    {
        Litelog(LEVEL_ERROR, "timerfd_create error!\n");
    }
//    Litelog(LEVEL_ERROR, "timerfd:%d!\n", tfd);
//
    fds->fd = tfd;
    fds->events = POLLIN;
//    Litelog(LEVEL_ERROR, "timerfd:%d!\n", fds->fd);
}

//TODO::这部分需要测试
void LiteDriver::SetOnceTimer(struct pollfd *fds, int first_time){
    struct timespec startTime, intervalTime;

    if(first_time == TURN){
        startTime.tv_sec = kFirstBumperTime;
        startTime.tv_nsec = 0;
    }else if(first_time == STRAIGHT){
        startTime.tv_sec = 0;
        startTime.tv_nsec = kRunTime;
    }
    //intervalTime.tv_sec = 0;
    //intervalTime.tv_nsec = 0;
    intervalTime.tv_sec = 0;
    intervalTime.tv_nsec = 0;		//原单位为纳秒，现为毫秒

    struct itimerspec newValue;
    newValue.it_value = startTime;
    newValue.it_interval = intervalTime;
    //设置超时时间，且为相对时间
    if (timerfd_settime(fds->fd, 0, &newValue, NULL) < 0)
    {
        Litelog(LEVEL_ERROR, "timerfd_settime error!\n");
    }
}

void LiteDriver::ProcessGyroAndOdoData(double time, void* pData){
    //void LiteDriver::ProcessGyroAndOdoData(void* pData){
//    Litelog(LEVEL_INFO, "Gyro&Odo data.\n");
    int skip_odo = false;
    if(g_stop_odo && !g_skip){
        BumperSet();
        g_skip = true;
        skip_odo = true;
//        return;
    }
    if(g_stop_odo){
        skip_odo = true;
//        return;
    }
    start->HandleImuAndOdometryMessage(trajectory_id, sensor_id_imu,
                                       sensor_id_odo, time, pData, skip_odo);
//        player_position3d_data_t gyroOdoData = *static_cast<player_position3d_data_t *>(pData);
//        Litelog(LEVEL_INFO, "Robot Pose(x,y,theta):(%f,%f,%f)\n",
//                gyroOdoData.pos.px,
//                gyroOdoData.pos.py,
//                gyroOdoData.pos.pz);
//        Litelog(LEVEL_INFO, "gyro value(roll,pitch,yaw):(%f,%f,%f)\n",
//                gyroOdoData.pos.proll,
//                gyroOdoData.pos.ppitch,
//                gyroOdoData.pos.pyaw);
//        Litelog(LEVEL_INFO, "Acc value(x,y,z):(%f,%f,%f)\n",
//                gyroOdoData.vel.px,
//                gyroOdoData.vel.py,
//                gyroOdoData.vel.pz);
//        Litelog(LEVEL_INFO, "odo value(left,right,v):(%d,%d,%lf)\n",
//                (int)gyroOdoData.vel.proll,
//                (int)gyroOdoData.vel.ppitch,
//                gyroOdoData.vel.pyaw);
}

//Test For New Odom Data
void LiteDriver::ProcessNewOdoData(double time, void* pData){
//    start->HandleOdometryMessage(trajectory_id, sensor_id_imu, time, pData);
}

//用于注册cartographer中trajectory_state回调使用
//DONE:下面这块函数应该进行一个封装 ,可以写一个设置速度的小程序在机器人上进行测试
//参数time的单位是s,目前的配置文件默认为5ms
void LiteDriver::SetTimer(struct pollfd *fds){
    int trigger_time = TRI_TIME;  //表示定时器时间
    int tfd = timerfd_create(CLOCK_REALTIME, TFD_NONBLOCK);
    if(tfd < 0)
    {
        Litelog(LEVEL_ERROR, "timerfd_create error!\n");
    }
//    Litelog(LEVEL_ERROR, "timerfd:%d!\n", tfd);
    struct timespec startTime,intervalTime;
    startTime.tv_sec = FIRST_TIME;
    startTime.tv_nsec = 0;                                //相当于立即到达超时时间
    //intervalTime.tv_sec = 0;                             //首次超时后,超时时间
    //intervalTime.tv_nsec = 0;
    if(trigger_time >= 1000)
    {
        intervalTime.tv_sec = (trigger_time / 1000);
    }
    //%1000 除去s
    intervalTime.tv_nsec = (trigger_time % 1000) * 1000000;		//原单位为纳秒，现为毫秒

    struct itimerspec newValue;
    newValue.it_value = startTime;
    newValue.it_interval = intervalTime;
    //设置超时时间，且为相对时间
    if (timerfd_settime(tfd, 0, &newValue, NULL) < 0)
    {
        Litelog(LEVEL_ERROR, "timerfd_settime error!\n");
    }
    fds->fd = tfd;
    fds->events = POLLIN;
}

void LiteDriver::Main(){

    //LOG(INFO)<<"------------------------Main---------------------------\n";

    //启动初始化
    Litelog(LEVEL_INFO, "Debug:%s----%d!\n", __FILE__, __LINE__);
    start->StartTrajectoryWithDefaultTopics(trajectory_options);

//    SetTimer(start_options.pose_publish_period_sec, &(fds[0]));
//    TRI_TIME = static_cast<int>(start_options.pose_publish_period_sec*1000);
    Litelog(LEVEL_INFO, "pose_publish_period_sec:%d!\n", TRI_TIME);
//    SetTimer(fds);
    SetTimer(&fds[0]);

    //此处注册接受数据的回调,将MainSetup中的回调提到此处完成，需验证是否可行:DONE
    RegisterProcessFun(PLAYER_MSGTYPE_DATA, PLAYER_LASER_DATA_SCANANGLE,
                       m_laser_addr, &LiteDriver::ProcessLaserData);
    RegisterProcessFun(PLAYER_MSGTYPE_DATA, PLAYER_POSITION3D_DATA_STATE,
                       m_gyro_odo_addr, &LiteDriver::ProcessGyroAndOdoData);
    //Test For New Odom Data
//    RegisterProcessFun(PLAYER_MSGTYPE_DATA, PLAYER_POSITION3D_DATA_STATE,
//                       m_new_odo_addr, &LiteDriver::ProcessNewOdoData);

    int ret;

    Litelog(LEVEL_INFO, "Lite Driver Main.\n");

    //FIXME::此处先用20s的sleep来等待laser的预热,已放到下面的循环中
//    sleep(20);

    //creat thread to set wheel speed
    pthread_create(&m_update_speed_thread, NULL, UpdateSpeedThread, this);


    ///用于接受pc端的控制命令
    char buf[MAXDATASIZE];
    struct sockaddr_in server;
    struct sockaddr_in client;
    socklen_t addrlen;
    if((listenfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    {
        Litelog(LEVEL_ERROR, "Creating  socket failed.");
        exit(1);
    }
    int opt = SO_REUSEADDR;
    setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    bzero(&server,sizeof(server));
    server.sin_family = AF_INET;
    server.sin_port = htons(PORT);
    server.sin_addr.s_addr= htonl (INADDR_ANY);
    if(bind(listenfd, (struct sockaddr *)&server, sizeof(server)) == -1) {
        Litelog(LEVEL_ERROR, "Binderror.");
        exit(1);
    }
    if(listen(listenfd,BACKLOG)== -1){  /* calls listen() */
        Litelog(LEVEL_ERROR, "listen()error\n");
        exit(1);
    }
    addrlen =sizeof(client);
    if((connectfd = accept(listenfd,(struct sockaddr*)&client,&addrlen))==-1) {
        Litelog(LEVEL_ERROR, "accept()error\n");
        exit(1);
    }
    Litelog(LEVEL_INFO, "Yougot a connection from cient's ip is %s, prot is %d\n",inet_ntoa(client.sin_addr),htons(client.sin_port));
    fds[1].fd = connectfd;
    fds[1].events = POLLIN;

//    ---------------------

    while(!m_stop_driver)
    {
        //下面的poll用于定时器
        ret = poll(fds, kFdNum, -1);
        if(ret < 0)
            Litelog(LEVEL_WARN, "Poll error!\n");
        ///此处使用一个标志位注册laser启动事件,然后还需要等待5s
        if(!LASER_START)
        {
            while(!LASER_START){
                Wait(0.1);
                ProcessMessages();
            }
            Litelog(LEVEL_WARN, "Laser start!\n");
            sleep(6);
        }

        if(fds[0].revents & POLLIN)   //处理数据
        {
            uint64_t howmany;
            if (read(fds[0].fd, &howmany, sizeof(howmany)) != sizeof(howmany))
            {
                Litelog(LEVEL_WARN, "read error!\n");
                continue;
            }
            //DONE:调用相应的处理函数
            start->PublishTrajectoeyStates();
        }

        if(fds[1].revents & POLLIN)   //处理按键
        {
            ssize_t num;
            if ((num = recv(fds[1].fd, buf, MAXDATASIZE, 0)) == -1)
            {
                Litelog(LEVEL_ERROR, "read error!\n");
                continue;
            }
            switch (buf[0]){
                case 'h':
                    SetSpeed(0.0, M_PI/9);
                    Litelog(LEVEL_INFO, "left!\n");
                    break;
                case 'l':
                    SetSpeed(0.0, -M_PI/9);
                    Litelog(LEVEL_INFO, "right!\n");
                    break;
                case 'j':
                    SetSpeed(0.0, M_PI/6);
                    Litelog(LEVEL_INFO, "acc left!\n");
                    break;
                case 'k':
                    SetSpeed(0.0, -M_PI/6);
                    Litelog(LEVEL_INFO, "acc right!\n");
                    break;
                case 's':
                    SetSpeed(0.15, 0.0);
                    Litelog(LEVEL_INFO, "straight!\n");
                    break;
                case 'a':
                    SetSpeed(0.2, 0.0);
                    Litelog(LEVEL_INFO, "accelerate!\n");
                    break;
                case 'q':
                    SetSpeed(0.0, 0.0);
                    Litelog(LEVEL_INFO, "pause!\n");
                    break;
                case 't':
                    SetSpeed(0.0, 0.0);
                    m_stop_driver = true;
                    Litelog(LEVEL_INFO, "terminate!\n");
                    break;
                default:
                    Litelog(LEVEL_ERROR, "command error!\n");
            }
        }
        if(fds[2].revents & POLLIN)    //处理碰撞
        {
            uint64_t howmany;
            if (read(fds[2].fd, &howmany, sizeof(howmany)) != sizeof(howmany))
            {
                Litelog(LEVEL_WARN, "read error!\n");
                continue;
            }
            //判断处理碰撞到哪一步
            if(++state_num%3 == 1){
                SetSpeed(0.2, 0.0);
                SetOnceTimer(&fds[2], STRAIGHT);
            }else if(state_num%3 == 2){
                SetSpeed(0.0, M_PI/9);
                SetOnceTimer(&fds[2], TURN);
            }else{
                SetSpeed(0.0, 0.0);
                g_stop_odo = false;
                g_skip = false;
            }
            state_num %= 3;
        }
        //FIXME:下面这个wait应该注释,是否应该也用一个定时器需要后面验证,否则触发频率太高导致计算不够
        Wait(0.1);
        ProcessMessages();
    }


    Litelog(LEVEL_INFO, "Lite Driver Main End.\n");
}

bool LiteDriver::SubscribeDevice(player_devaddr_t& addr,
                                 ConfigFile* cf, int section,
                                 int32_t interf_code,
                                 int32_t index,
                                 const char *key,
                                 Device** ppDevice){
    Device* pDevice = nullptr;

    memset(&addr, 0, sizeof(player_devaddr_t));

    if (cf->ReadDeviceAddr(&addr, section, "requires", interf_code, index, key))
    {
        Litelog(LEVEL_ERROR, "Device[%d:%d:%s] is required\n", interf_code, index, key);
        return false;
    }

    if (!(pDevice = deviceTable->GetDevice(addr)))
    {
        Litelog(LEVEL_ERROR, "Unable to locate suitable device[%d:%d]\n", interf_code, index);
        return false;
    }

    if (pDevice->Subscribe(InQueue) != 0)
    {
        Litelog(LEVEL_ERROR, "Unable to subscribe to device[%d:%d]\n", interf_code, index);
        return false;
    }

    if (ppDevice != nullptr)
    {
        (*ppDevice) = pDevice;
    }

    return true;
}

bool LiteDriver::AddInterface(player_devaddr_t& addr,
                              ConfigFile* cf,
                              int section,
                              int32_t interf_code,
                              int32_t index,
                              const char *key){
    if (!cf->ReadDeviceAddr(&addr, section, "provides", interf_code, index, key))
    {
        return (ThreadedDriver::AddInterface(addr) == 0);
    }
    return false;
}

void LiteDriver::SetSpeed(double lineSpeed, double yawSpeed, bool force){
    Litelog(LEVEL_INFO, "Set speed  %f, %f.\n", lineSpeed, yawSpeed);
    m_line_speed = lineSpeed;
    m_yaw_speed = yawSpeed;
    m_force = force;
}

void LiteDriver::SendSpeedCMD(double lineSpeed, double yawSpeed, bool force){

    if(force){
        player_position2d_cmd_vel_t vel_cmd;
        vel_cmd.state = 1;
        vel_cmd.vel.px = lineSpeed;
        vel_cmd.vel.py = 0;
        vel_cmd.vel.pa = yawSpeed;

        m_pWheelDevice->PutMsg(
                this->InQueue,
                PLAYER_MSGTYPE_CMD, PLAYER_POSITION2D_CMD_VEL,
                (void*)&vel_cmd, sizeof(vel_cmd), NULL);
    }else{
        player_position2d_cmd_vel_head_t vel_head_cmd;
        vel_head_cmd.velocity = lineSpeed;
        vel_head_cmd.angle = yawSpeed;

        m_pWheelDevice->PutMsg(
                this->InQueue,
                PLAYER_MSGTYPE_CMD, PLAYER_POSITION2D_CMD_VEL_HEAD,
                (void*)&vel_head_cmd, sizeof(vel_head_cmd), NULL);
    }
}

void LiteDriver::UpdateSpeed(){
    Litelog(LEVEL_INFO, "Update Speed thread start.\n");
    while(!m_stop_driver){
        std::this_thread::sleep_for(std::chrono::milliseconds(WHEEL_SPEED_UPDATE_INTERVAL));
        SendSpeedCMD(m_line_speed, m_yaw_speed);
    }
}

void LiteDriver::SaveMap(MapType maptype, char const* path){
    double resolution = 0.05;  //格子的分辨率，5cm
    //获取所有子图
    std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice> submap_slices;
    for (const auto& submap_id_data: start->map_builder_bridge()->
            map_builder()->pose_graph()->GetAllSubmapData()) {
        ::cartographer::mapping::proto::SerializedData proto;
        auto* submap_proto = proto.mutable_submap();
        submap_proto->mutable_submap_id()->set_trajectory_id(submap_id_data.id.trajectory_id);
        submap_proto->mutable_submap_id()->set_submap_index(submap_id_data.id.submap_index);
        submap_id_data.data.submap->ToProto(submap_proto, true);

        const ::cartographer::mapping::SubmapId id{
            submap_id_data.id.trajectory_id,
            submap_id_data.id.submap_index
        };

        //此处获得的子图pose已经转换过，是相对于全局的
        FillSubmapSlice(submap_id_data.data.pose, *submap_proto, &submap_slices[id]);

    }

    //拼接子图
    auto result = ::cartographer::io::PaintSubmapSlices(submap_slices, resolution);

//    ::cartographer::io::StreamFileWriter pgm_writer("test.pgm");
//    ::cartographer::io::Image image(std::move(result.surface));
    {
//        const std::string header = "P5\n# Cartographer map; " +
//                                   std::to_string(resolution) + " m/pixel\n" +
//                                   std::to_string(image.width()) + " " +
//                                   std::to_string(image.height()) + "\n255\n";
//        pgm_writer.Write(header.data(), header.size());
//
//        std::ostringstream info;
//        for (int y = 0; y < image.height(); ++y) {
//            for (int x = 0; x < image.width(); ++x) {
//                const char color = image.GetPixel(x, y)[0];
//                pgm_writer.Write(&color, 1);
//                info << (int)color << " ";
//            }
//            info << std::endl;
//        }
//        std::ofstream ofstreamer("map.txt");
//        if(!ofstreamer)
//            return;
//        ofstreamer << info.str();
//        ofstreamer << "height:" << image.height() << std::endl;
//        ofstreamer << "width:" << image.width() << std::endl;
//        ofstreamer.close();


//        LOG(INFO) << info.str();
//        LOG(INFO) << image.height();
//        LOG(INFO) << image.width();

//
//        const Eigen::Vector2d origin(
//                -result.origin.x() * resolution,
//                (result.origin.y() - image.height()) * resolution);
//
//        ::cartographer::io::StreamFileWriter yaml_writer("test.yaml");
//        const std::string output =
//                "image: " + pgm_writer.GetFilename() + "\n" +
//                "resolution: " + std::to_string(resolution) + "\n" + "origin: [" +
//                std::to_string(origin.x()) + ", " + std::to_string(origin.y()) +
//                ", 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
//        yaml_writer.Write(output.data(), output.size());

        //重置地图
        memset(*grid_map.occupied_value, 0, sizeof(rock_occupied_value_t)*1024*1024);

        {   //开始填地图
            //坐标原点
            const int origin_x = result.origin.x();  //是否从0开始计算
            const int origin_y = result.origin.y();
//            auto result = ::cartographer::io::PaintSubmapSlices(submap_slices, resolution);
            const int width = cairo_image_surface_get_width(result.surface.get());   //获取拼好后地图的宽和高
            const int height = cairo_image_surface_get_height(result.surface.get());
            const uint32_t* pixel_data = reinterpret_cast<uint32_t*>(
                    cairo_image_surface_get_data(result.surface.get()));

            //坐标最值的记录，用于裁剪地图
            grid_map.region.size.width =  width;
            grid_map.region.size.height =  height;
            grid_map.region.point.x.grid = -origin_x;
            grid_map.region.point.y.grid = -origin_y;

            //FREE:255,OBSTACLE:1,UNKNOWN:0
            int value = 0;

            //数组坐标系的起始位置
            int arr_x = -origin_x + 512;
            int arr_y = -origin_y + 512;
            LOG(INFO) << "origin_x:" << origin_x << std::endl;
            LOG(INFO) << "origin_y:" << origin_y << std::endl;
            LOG(INFO) << "height:" << height << std::endl;
            LOG(INFO) << "width:" << width << std::endl;

            //数组中的一维坐标
            int arr_final;
            rock_occupied_value_t *arr_add = static_cast<rock_occupied_value_t*>(*grid_map.occupied_value);
            int local_start = 0;

            std::ostringstream info1;

            for (int y = 0; y < height; ++y) {

                arr_final = arr_y * 1024 + arr_x;     //计算位置
                ++arr_y;

                for (int x = 0; x < width; ++x) {
                    const uint32_t packed = pixel_data[y * width + x];
                    const unsigned char color = packed >> 16;
                    const unsigned char observed = packed >> 8;
                    if(observed == 0){
                        value = 0;
                    }else {
                        value = ::cartographer::common::RoundToInt((1. - color / 255.) * 100.);
                        if(value <= 50){   //TODO:区分障碍还是无障碍的阈值，还可以再调整。
                            value = 255;
                        }else {
                            value = 1;
                        }
                    }
                    //所填的值先转换到地图坐标系，再转换到数组坐标系
//                    LOG(INFO) << __LINE__ << ":" << arr_final << std::endl;
                    //可以在这将地图分为两种填充
                    //全部地图
                    arr_add[arr_final++] = static_cast<rock_occupied_value_t >(value);
                    //已使用地图
//                    arr_add[local_start++] = static_cast<rock_occupied_value_t >(value);


                    info1 << std::setw(3) << value << " ";
                }
                info1 << std::endl;
            }
            info1 << "width:" << width << std::endl;
            info1 << "height:" << height << std::endl;
            info1 << "origin(x,y):" << result.origin.x() << ',' << result.origin.y() << std::endl;

            info1 << std::endl;
            std::ofstream ofstreamer1("grid.txt");
            if(!ofstreamer1)
                return;
            ofstreamer1 << info1.str();
            ofstreamer1.close();
        }

        {
            std::ostringstream info_map;

            //这部分应在上面进行裁剪，将地图按照已使用，全部，更新进行提供
            for (int i = 0; i < 1024; ++i) {
                for (int j = 0; j < 1024; ++j) {
                    info_map << std::setw(3) << (int)grid_map.occupied_value[i][j] << " ";
                }
                info_map << std::endl;
            }
            std::ofstream ofstream_allmap("map_all.txt");
            if(!ofstream_allmap)
                return;
            ofstream_allmap << info_map.str();
            ofstream_allmap.close();
        }

        {
            std::ostringstream info_usemap;
            for (int i = grid_map.region.point.y.grid + 512; i < grid_map.region.point.y.grid + 512 +
                    grid_map.region.size.height; ++i) {
                for (int j = grid_map.region.point.x.grid + 512; j < grid_map.region.point.x.grid + 512 +
                        grid_map.region.size.width; ++j) {
                    info_usemap << std::setw(3) << (int)grid_map.occupied_value[i][j] << " ";
                }
                info_usemap << std::endl;
            }
            std::ofstream ofstream_usedmap("map_used.txt");
            if(!ofstream_usedmap)
                return;
            ofstream_usedmap << info_usemap.str();
            ofstream_usedmap.close();
        }

//            std::ofstream ofstream_updatemap("map_update.txt");
//            if(!ofstream_updatemap)
//                return;
//            ofstream_updatemap << info_map.str();
//            ofstream_updatemap.close();
    }
    //填充地图
}

//修改submap的构造，不使用proto
//此方法用于处理submap,使之成为可以拼接的状态
void LiteDriver::FillSubmapSlice(
        const ::cartographer::transform::Rigid3d& global_submap_pose,
        const ::cartographer::mapping::proto::Submap& proto,
        ::cartographer::io::SubmapSlice* submap_slice){
    ::cartographer::mapping::proto::SubmapQuery::Response response;
    ::cartographer::transform::Rigid3d local_pose;
    ::cartographer::mapping::Submap2D submap(proto.submap_2d());
    submap.ToResponseProto(global_submap_pose, &response);
    submap_slice->pose = global_submap_pose;

    auto& texture_proto = response.textures(0);
    const ::cartographer::io::SubmapTexture::Pixels pixels = ::cartographer::io::UnpackTextureData(
            texture_proto.cells(), texture_proto.width(), texture_proto.height());
    submap_slice->width = texture_proto.width();
    submap_slice->height = texture_proto.height();
    submap_slice->resolution = texture_proto.resolution();
    submap_slice->slice_pose =
            ::cartographer::transform::ToRigid3(texture_proto.slice_pose());
    submap_slice->surface =
            ::cartographer::io::DrawTexture(pixels.intensity, pixels.alpha, texture_proto.width(),
                                            texture_proto.height(), &submap_slice->cairo_data);
}

//获取全局位姿
::cartographer::transform::Rigid3d LiteDriver::getCurGlobalPose(const int trajectory_id)
{
    return start->map_builder_bridge()->map_builder()->pose_graph()->GetLocalToGlobalTransform(trajectory_id);
}
//局部位姿
::cartographer::transform::Rigid3d LiteDriver::getCurLocalPose()
{
    return start->map_builder_bridge()->GetTrajectoryStates().begin()->second.local_slam_data->local_pose;
}

//北京公司要求接口
//创建slam句柄
rock_slam_t LiteDriver::_rock_slam_create(RockSlamSystem system)
{
    rock_slam_t myslam =(rock_slam_t)malloc(sizeof(rock_slam));

    if(system == ROCK_SLAM_SYSTEM_LASER)
    {
        myslam->trajectory_id = 0;
        new(&(myslam->sensor_id_laser))std::string;
        new(&(myslam->sensor_id_multi_laser))std::string;
        new(&(myslam->sensor_id_imu))std::string;
        new(&(myslam->sensor_id_odo))std::string;

        myslam->sensor_id_laser = "scan";
        myslam->sensor_id_multi_laser = "echoes";
        myslam->sensor_id_imu = "imu";
        myslam->sensor_id_odo = "odom";

    }

    return myslam;
}
//释放slam句柄
void LiteDriver::_rock_slam_release(rock_slam_t* slam)
{
    free(&slam);
}

//move方法向slam传入odo和gyro数据
void LiteDriver::_rock_slam_move(rock_slam_t slam,rock_motion_t const* motion)
{

}

//observe方法向slam传入laser数据
void LiteDriver::_rock_slam_observe(rock_slam_t slam, rock_observation_t const* observation)
{

}

//设置日志函数
void LiteDriver::_rock_slam_set_logger(rock_logger_t logger)
{
    mylogfunc = logger;
}

//设置日志级别
void LiteDriver::_rock_slam_set_log_level(uint32_t level)
{
    log_level = level;
}

//获取当前地图上已使用的区域
void LiteDriver::_rock_slam_map_used(rock_slam_t slam, rock_rect_t* rect) {
    if(lockflag)
    {
        rect = rect_lock;  //直接返回保存在本地的数据

        //测试代码，将结果保存在文件中方便查看
        {
            std::ostringstream info;

            info << "rect size width = "<< rect->size.width << "\n";
            info << "rect size height = "<< rect->size.height << "\n";
            info << "rect point x = "<< rect->point.x.grid << "\n";
            info << "rect point y = "<< rect->point.y.grid << "\n";

            std::ofstream ofstream_usedmap("map_used_rect_lock.txt");
            if(!ofstream_usedmap)
                return;
            ofstream_usedmap << info.str();
            ofstream_usedmap.close();
        }

        return ;
    }


    double resolution = 0.05;  //格子的分辨率，5cm
    //获取所有子图
    std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice> submap_slices;
    for (const auto &submap_id_data: start->map_builder_bridge()->
            map_builder()->pose_graph()->GetAllSubmapData()) {
        ::cartographer::mapping::proto::SerializedData proto;
        auto *submap_proto = proto.mutable_submap();
        submap_proto->mutable_submap_id()->set_trajectory_id(submap_id_data.id.trajectory_id);
        submap_proto->mutable_submap_id()->set_submap_index(submap_id_data.id.submap_index);
        submap_id_data.data.submap->ToProto(submap_proto, true);

        const ::cartographer::mapping::SubmapId id{
                submap_id_data.id.trajectory_id,
                submap_id_data.id.submap_index
        };

        //此处获得的子图pose已经转换过，是相对于全局的
        FillSubmapSlice(submap_id_data.data.pose, *submap_proto, &submap_slices[id]);

    }

    //拼接子图
    auto result = ::cartographer::io::PaintSubmapSlices(submap_slices, resolution);
    //坐标原点
    const int origin_x = result.origin.x();  //是否从0开始计算
    const int origin_y = result.origin.y();
//            auto result = ::cartographer::io::PaintSubmapSlices(submap_slices, resolution);
    const int width = cairo_image_surface_get_width(result.surface.get());   //获取拼好后地图的宽和高
    const int height = cairo_image_surface_get_height(result.surface.get());
    const uint32_t *pixel_data = reinterpret_cast<uint32_t *>(
            cairo_image_surface_get_data(result.surface.get()));

    //坐标最值的记录，用于裁剪地图
    rect->size.width = width;
    rect->size.height = height;
    rect->point.x.grid = -origin_x;
    rect->point.y.grid = -origin_y;


    //测试代码，将结果保存在文件中方便查看
    {
        std::ostringstream info;

        info << "rect size width = "<< rect->size.width << "\n";
        info << "rect size height = "<< rect->size.height << "\n";
        info << "rect point x = "<< rect->point.x.grid << "\n";
        info << "rect point y = "<< rect->point.y.grid << "\n";

        std::ofstream ofstream_usedmap("map_used_rect.txt");
        if(!ofstream_usedmap)
            return;
        ofstream_usedmap << info.str();
        ofstream_usedmap.close();
    }

//    std::ostringstream info;
//
//    info << "rect size width = "<< rect->size.width << "\n";
//    info << "rect size height = "<< rect->size.height << "\n";
//    info << "rect point x = "<< rect->point.x.grid << "\n";
//    info << "rect point y = "<< rect->point.y.grid << "\n";
//
//    std::ofstream ofstream_usedmap("map_used_rect.txt");
//    if(!ofstream_usedmap)
//        return;
//    ofstream_usedmap << info.str();
//    ofstream_usedmap.close();

}

//获取当前完整三值（unknown，free，obstacle）地图的一维数组
void LiteDriver::_rock_slam_map(rock_slam_t slam, rock_occupied_value_t* buffer, rock_size_t size)
{

    //LOG(INFO)<<"------------------------_rock_slam_map---------------------------\n";

    if(lockflag)
    {
        memset(buffer,0, sizeof(size));
        buffer = gridmap_lock;      //直接返回板寸在本地的地图数据

        //测试代码：
        //将保存的数据存储到文件中方便查看
        {
            //memset(buffer, 0, sizeof(size));

            std::ostringstream info_map;

            //这部分应在上面进行裁剪，将地图按照已使用，全部，更新进行提供
            for (int i = 0; i < 1024; ++i) {
                for (int j = 0; j < 1024; ++j) {
                    info_map << std::setw(3) << (int)buffer[i*1024+j] << " ";
                }
                info_map << std::endl;
            }
            std::ofstream ofstream_allmap("map_all_buffer_lock.txt");
            if(!ofstream_allmap)
                return;
            ofstream_allmap << info_map.str();
            ofstream_allmap.close();
        }

        return ;
    }


    rock_rect_t* rect = (rock_rect_t*)malloc(sizeof(rock_rect_t));

    double resolution = 0.05;  //格子的分辨率，5cm
    //获取所有子图
    std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice> submap_slices;
    for (const auto& submap_id_data: start->map_builder_bridge()->
            map_builder()->pose_graph()->GetAllSubmapData()) {
        ::cartographer::mapping::proto::SerializedData proto;
        auto* submap_proto = proto.mutable_submap();
        submap_proto->mutable_submap_id()->set_trajectory_id(submap_id_data.id.trajectory_id);
        submap_proto->mutable_submap_id()->set_submap_index(submap_id_data.id.submap_index);
        submap_id_data.data.submap->ToProto(submap_proto, true);

        const ::cartographer::mapping::SubmapId id{
                submap_id_data.id.trajectory_id,
                submap_id_data.id.submap_index
        };

        //此处获得的子图pose已经转换过，是相对于全局的
        FillSubmapSlice(submap_id_data.data.pose, *submap_proto, &submap_slices[id]);

    }

    //拼接子图
    auto result = ::cartographer::io::PaintSubmapSlices(submap_slices, resolution);
    {
        //重置地图
        memset((rock_occupied_value_t *)buffer, 0, sizeof(size));

        {   //开始填地图
            //坐标原点
            const int origin_x = result.origin.x();  //是否从0开始计算
            const int origin_y = result.origin.y();
//            auto result = ::cartographer::io::PaintSubmapSlices(submap_slices, resolution);
            const int width = cairo_image_surface_get_width(result.surface.get());   //获取拼好后地图的宽和高
            const int height = cairo_image_surface_get_height(result.surface.get());
            const uint32_t* pixel_data = reinterpret_cast<uint32_t*>(cairo_image_surface_get_data(result.surface.get()));

            //坐标最值的记录，用于裁剪地图
            rect->size.width =  width;
            rect->size.height =  height;
            rect->point.x.grid = -origin_x;
            rect->point.y.grid = -origin_y;

            //FREE:255,OBSTACLE:1,UNKNOWN:0
            int value = 0;

            //数组坐标系的起始位置
            int arr_x = -origin_x + 512;
            int arr_y = -origin_y + 512;
            LOG(INFO) << "origin_x:" << origin_x << std::endl;
            LOG(INFO) << "origin_y:" << origin_y << std::endl;
            LOG(INFO) << "height:" << height << std::endl;
            LOG(INFO) << "width:" << width << std::endl;

            //数组中的一维坐标
            int arr_final;
//            rock_occupied_value_t *arr_add = static_cast<rock_occupied_value_t*>(*grid_map.occupied_value);
            int local_start = 0;

            std::ostringstream info1;

            for (int y = 0; y < height; ++y) {

                arr_final = arr_y * 1024 + arr_x;
                ++arr_y;

                for (int x = 0; x < width; ++x) {
                    const uint32_t packed = pixel_data[y * width + x];
                    const unsigned char color = packed >> 16;
                    const unsigned char observed = packed >> 8;
                    if(observed == 0){
                        value = 0;
                    }else {
                        value = ::cartographer::common::RoundToInt((1. - color / 255.) * 100.);
                        if(value <= 50){   //TODO:区分障碍还是无障碍的阈值，还可以再调整。
                            value = 255;
                        }else {
                            value = 1;
                        }
                    }
                    //所填的值先转换到地图坐标系，再转换到数组坐标系
//                    LOG(INFO) << __LINE__ << ":" << arr_final << std::endl;
                    //可以在这将地图分为两种填充
                    //全部地图
                    buffer[arr_final++] = static_cast<rock_occupied_value_t >(value);

                }

            }

        }

        //测试代码：
        //将保存的数据存储到文件中方便查看
        {
            //memset(buffer, 0, sizeof(size));

            std::ostringstream info_map;

            //这部分应在上面进行裁剪，将地图按照已使用，全部，更新进行提供
            for (int i = 0; i < 1024; ++i) {
                for (int j = 0; j < 1024; ++j) {
                    info_map << std::setw(3) << (int)buffer[i*1024+j] << " ";
                }
                info_map << std::endl;
            }
            std::ofstream ofstream_allmap("map_all_buffer.txt");
            if(!ofstream_allmap)
                return;
            ofstream_allmap << info_map.str();
            ofstream_allmap.close();
        }
    }

}

//锁地图
void LiteDriver::_rock_slam_lock(rock_slam_t slam, bool lock)
{
    if(lockflag == lock)
    {
        return ;
    }else{

        if(lock)
        {
            _rock_slam_map_used(slam,rect_lock);             //存下锁住地图时刻的已使用区域和地图
            _rock_slam_map(slam,gridmap_lock,1024*1024);

        }else{
            memset(gridmap_lock,0,sizeof(1024*1024));       //清空保存在本地的地图
        }

        lockflag = lock;    //必须先存副本再置锁标志位

        return ;
    }

}

//查询地图是否已锁
void LiteDriver::_rock_slam_locked(rock_slam_t slam, bool* locked)
{
    *locked = lockflag;
}

void* UpdateSpeedThread(void* obj){
    LiteDriver * this_driver = (LiteDriver * )obj;
    this_driver -> UpdateSpeed();
    return nullptr;
}

extern "C" {
int player_driver_init(DriverTable *table) {
    Litelog(LEVEL_INFO, "player_driver_init for lite driver.\n");
    driverTable->AddDriver("shadow_slam", [](ConfigFile *cf, int section) {
        return static_cast<Driver *>(new LiteDriver(cf, section));
    });
    return 0;
}
}
