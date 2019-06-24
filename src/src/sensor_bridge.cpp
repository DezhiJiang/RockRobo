//
// Created by root on 5/10/18.
//

#include "sensor_bridge.h"

#include "cartographer/common/make_unique.h"
#include "utility.h"

#include <iostream>

namespace RockRobo{
    namespace carto = ::cartographer;

    using carto::transform::Rigid3d;

    constexpr int Laser_Intensity_Mask = 0xFFF;
    constexpr int Laser_Invalid_Mask = 0xF000;
    constexpr float Time_Increment = 0.0;   //FIXME::现在将其固定为0.0

    ///用于是否第一次收到Imu和Odom数据,只记录,第一次不进行添加,之后更新这个值
    double g_imu_odom_time = 0;
    player_position3d_data_t g_imu_odom_data;
//    namespace {
//        const std::string& CheckNoLeadingSlash(const std::string& frame_id){
//            if(frame_id.size() > 0){
//                CHECK_NE(frame_id[0], '/'a) << "The frame _id" << frame_id
//                                            << " should not start whit a /.";
//            }
//            return frame_id;
//        }
//    } //namespace

    SensorBridge::SensorBridge(
            const int num_subdivisions_per_laser_scan,
//            const std::string& tracking_frame,
//            const double lookup_transform_timeout_sec,
            carto::mapping::TrajectoryBuilderInterface* const trajectory_builder
    ) : num_subdivisions_per_laser_scan_(num_subdivisions_per_laser_scan),
        trajectory_builder_(trajectory_builder){}

    void SensorBridge::HandleOdometryMessage(const std::string& sensor_id,
                                             double time,
                                             void *pData){
        std::unique_ptr<carto::sensor::OdometryData> odometry_data = ToOdometryData(time, pData);
        //Test For New Odometry Data
        player_position3d_data_t odoData = *static_cast<player_position3d_data_t *> (pData);
        //rigid3d 数据在构造以后不能修改？？？以下访问方式是错误的
//        odometry_data->pose.translation_[] = odoData.pos.ppitch;
//        odometry_data->pose.translation().y() = odoData.pos.pyaw;
        //End
        trajectory_builder_->AddSensorData(sensor_id,
                                           carto::sensor::OdometryData{odometry_data->time, odometry_data->pose});
    }

    //将Imu和Odometry信息一起处理，因为这两个数据一起得到
    void SensorBridge::HandleImuAndOdometryMessage(const std::string& sensor_id_imu,
                                                   const std::string& sensor_id_odo,
                                                   double time,
                                                   void *pData, int skip_odo){
        if(g_imu_odom_time == 0){
            g_imu_odom_time = time;
            g_imu_odom_data = *static_cast<player_position3d_data_t *> (pData);
            return;
        }
        std::unique_ptr<carto::sensor::ImuData> imu_data = ToImuData(time, pData);

//        std::cout << imu_data->linear_acceleration << "---"
//                  << imu_data->angular_velocity << std::endl;

        trajectory_builder_->AddSensorData(sensor_id_imu,
                                           carto::sensor::ImuData{imu_data->time,
                                                                  imu_data->linear_acceleration,
                                                                  imu_data->angular_velocity});

        if(!skip_odo){
            std::unique_ptr<carto::sensor::OdometryData> odometry_data = ToOdometryData(time, pData);
            //New Add Way
            trajectory_builder_->AddSensorData(
                    sensor_id_odo,
                    carto::sensor::OdometryData{odometry_data->time, odometry_data->pose});
        }
        g_imu_odom_time = time;
        g_imu_odom_data = *static_cast<player_position3d_data_t *> (pData);
    }

    void SensorBridge::HandleLaserScanMessage(const std::string& sensor_id,
                                              double time,
                                              void *pData){
        carto::sensor::PointCloudWithIntensities point_cloud;
        carto::common::Time timestamp;
        std::tie(point_cloud, timestamp) = ToPointCloudWithIntensities(time, pData); ///notice :: this warning can be ignored

        HandleLaserScan(sensor_id, timestamp, point_cloud);
    }

    void SensorBridge::HandleMultiEchoLaserScanMessage(const std::string& sensor_id,
                                                       double time,
                                                       void *pData){
        carto::sensor::PointCloudWithIntensities point_cloud;
        carto::common::Time timestamp;
        std::tie(point_cloud, timestamp) = ToPointCloudWithIntensities(time, pData);

        HandleLaserScan(sensor_id, timestamp, point_cloud);
    }

    std::unique_ptr<::cartographer::sensor::ImuData> SensorBridge::ToImuData(double time, void *pData){
        const carto::common::Time timestamp = FromDouble(time);
        //TODO: 此处tf信息使用与laser数据相同的方式，需要测试
        const auto sensor_to_tracking = ::cartographer::common::make_unique<
                Rigid3d >(Rigid3d(Eigen::Vector3d(0,0,0), Eigen::Quaterniond(1,0,0,0)));
//                Rigid3d >(Rigid3d(Eigen::Vector3d(0,0,0), Eigen::Quaterniond(0.999987, -0.00414793,
//                                                                             0.00178995, 0.00251686)));
        player_position3d_data_t imuData = *static_cast<player_position3d_data_t *> (pData);
        //! player平台获取的数据中没有协方差矩阵，这个应该没有实际影响
        ///此处要注意欧拉角的xyz是否对应
//        Litelog(LEVEL_INFO, "Imu time:%f\n", time);
        player_position3d_data_t imu_computed_data;
        double interval_time = time - g_imu_odom_time;
        CHECK_GE(interval_time, 0);
//        imu_computed_data.vel.px = (imuData.vel.px - g_imu_odom_data.vel.px) / interval_time;
//        imu_computed_data.vel.py = (imuData.vel.py - g_imu_odom_data.vel.py) / interval_time;
//        imu_computed_data.vel.pz = (imuData.vel.pz - g_imu_odom_data.vel.pz) / interval_time;
        imu_computed_data.vel.proll = (imuData.vel.proll - g_imu_odom_data.vel.proll) / interval_time;
        imu_computed_data.vel.ppitch = (imuData.vel.ppitch - g_imu_odom_data.vel.ppitch) / interval_time;
        imu_computed_data.vel.pyaw = (imuData.vel.pyaw - g_imu_odom_data.vel.pyaw) / interval_time;

        static int printi = 0;
        if(printi++ % 100 == 0){
//            Litelog(LEVEL_INFO, "Imu ang_v data:%lf*****%lf****%lf\n", imuData.pos.proll, imuData.pos.ppitch, imuData.pos.pyaw);
//            Litelog(LEVEL_INFO, "Imu line_a data:%lf****%lf****%lf\n", imuData.vel.px, imuData.vel.py, imuData.vel.pz);
//            Litelog(LEVEL_INFO, "Imu line_a pyaw data:%lf\n", imu_computed_data.vel.pyaw);
        }

        return carto::common::make_unique<carto::sensor::ImuData>(
                carto::sensor::ImuData{
                        timestamp,
                        sensor_to_tracking->rotation() *
                        Eigen::Vector3d(imuData.vel.px, imuData.vel.py, imuData.vel.pz),
                        sensor_to_tracking->rotation() *
                        Eigen::Vector3d(imu_computed_data.pos.proll,
                                        imu_computed_data.pos.ppitch, imu_computed_data.pos.pyaw)});
    }

    std::unique_ptr<::cartographer::sensor::OdometryData> SensorBridge::ToOdometryData(double time, void *pData){
        const carto::common::Time timestamp = FromDouble(time);
        //TODO: 此处tf信息使用与laser数据相同的方式，需要测试
        //TODO: sensor tracking 是相同的，可以提出来作为一个全局变量减小计算量，以及下面的计算也可以先计算好。
        const auto sensor_to_tracking = ::cartographer::common::make_unique<
                Rigid3d >(Rigid3d(Eigen::Vector3d(0,0,0), Eigen::Quaterniond(1,0,0,0)));
        player_position3d_data_t odoData = *static_cast<player_position3d_data_t *> (pData);
        ///WARN::此处需要将欧拉角转化为四元数, 通过Eigen库中先转换为旋转矩阵，再转化为四元数
        ///此处要注意欧拉角的xyz是否对应
        Eigen::Vector3d euler(odoData.pos.pyaw, odoData.pos.ppitch, odoData.pos.proll);
        //Eigen::Matrix3d R;   //可以不经过旋转矩阵的转换
        Eigen::Quaterniond qua = Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitZ())
                                 * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY())
                                 * Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitX());

//        Litelog(LEVEL_INFO, "Odom quaterniond:(%lf,%lf,%lf,%lf)\n", qua.x(), qua.y(), qua.z(), qua.w());
        return carto::common::make_unique<carto::sensor::OdometryData>(
                carto::sensor::OdometryData{
                        timestamp, Rigid3d({odoData.pos.px, odoData.pos.py, odoData.pos.pz},
                                           qua) * sensor_to_tracking->inverse()});
    }

    //这个函数是将player的laser数据转化为cartographer数据，其中角度的转化上有区别，每个angle与一个intensity对应
    //FIXME::需要看angle、range与intensity是否对应,重点应该是数量
    std::tuple<::cartographer::sensor::PointCloudWithIntensities,
            ::cartographer::common::Time> SensorBridge::ToPointCloudWithIntensities(double time, void *pData){
        carto::sensor::PointCloudWithIntensities point_cloud;
        player_laser_data_scanangle laserData = *static_cast<player_laser_data_scanangle *>(pData);
        CHECK_EQ(laserData.intensity_count, laserData.ranges_count);
        for (int i = 0; i < laserData.ranges_count; ++i) {
            if((uint16_t)laserData.intensity[i] & Laser_Invalid_Mask){    //如果高位标志位有一位置位，表示数据不正确
                continue;
            }else if(i == laserData.angles_count || i == laserData.intensity_count){  //判断是否越界
                break;
            } else{
                const auto& range = laserData.ranges[i];
                if(range <= laserData.max_range){
                    const auto& angle = laserData.angles[i];
                    const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
                    Eigen::Vector4f point;
                    point << rotation * (range * Eigen::Vector3f::UnitX()),
                            i * Time_Increment;
                    point_cloud.points.push_back(point);
                    const auto& intensity = (uint16_t)laserData.intensity[i] & Laser_Intensity_Mask;
                    point_cloud.intensities.push_back(intensity);
                }else{
                    continue;
                }
            }
        }
        ///需要验证下面第二个是否正确
        carto::common::Time timestamp = FromDouble(time);
//        Litelog(LEVEL_INFO, "-----------time:%f\n", time);
//        carto::common::Time timestamp = cartographer::common::FromSeconds(time); ///对应事件类型不对，应该不能转换
        if(!point_cloud.points.empty()){    ///这一步骤可能是不需要的，因为timeincrement为0
            const double duration = point_cloud.points.back()[3];
            timestamp += cartographer::common::FromSeconds(duration);
            for(Eigen::Vector4f& point : point_cloud.points){
                point[3] -= duration;
            }
        }
        return std::make_tuple(point_cloud, timestamp);
    }

    void SensorBridge::HandleLaserScan(const std::string& sensor_id,
                                       const ::cartographer::common::Time time,
                                       const ::cartographer::sensor::PointCloudWithIntensities& points){
        ///new version code
        if(points.points.empty()){
            return;
        }
        ///new version code end
        CHECK_LE(points.points.back()[3], 0);
        for (int i = 0; i != num_subdivisions_per_laser_scan_; ++i){
            const size_t start_index = points.points.size() * i / num_subdivisions_per_laser_scan_;
            const size_t end_index = points.points.size() * (i + 1) / num_subdivisions_per_laser_scan_;
            carto::sensor::TimedPointCloud subdivision(points.points.begin() + start_index,
                                                       points.points.begin() + end_index);
            if(start_index == end_index){
                continue;
            }

            const double time_to_subdivision_end = subdivision.back()[3];

            const carto::common::Time subdivision_time =
                    time + carto::common::FromSeconds(time_to_subdivision_end);
            auto it = sensor_to_previous_subdivision_time_.find(sensor_id);
            if(it != sensor_to_previous_subdivision_time_.end() &&
               it->second >= subdivision_time){  // this waring can be ignored
                LOG(INFO) << "Ignored subdivision of a LaserScan message from sensor "
                          << sensor_id << " because previous subdivision trme "
                          << it->second << " is not before current subdivision time "
                          << subdivision_time;
                continue;
            }
            sensor_to_previous_subdivision_time_[sensor_id] = subdivision_time;

            for(Eigen::Vector4f& point: subdivision){
                point[3] -= time_to_subdivision_end;
            }
            CHECK_EQ(subdivision.back()[3], 0);
            HandleRangefinder(sensor_id, subdivision_time, subdivision);
        }
    }

    void SensorBridge::HandleRangefinder(const std::string& sensor_id,
                                         const ::cartographer::common::Time time,
                                         const ::cartographer::sensor::TimedPointCloud& ranges){
        const auto sensor_to_tracking = ::cartographer::common::make_unique<
                Rigid3d >(Rigid3d(Eigen::Vector3d(0,0,0), Eigen::Quaterniond(1,0,0,0)));
//                Rigid3d >(Rigid3d(Eigen::Vector3d(0,0,0), Eigen::Quaterniond(0.999994, 0.00286292,
//                                                                             -0.00109758, 0.00178705)));
        trajectory_builder_->AddSensorData(sensor_id, carto::sensor::TimedPointCloudData{
                time, sensor_to_tracking->translation().cast<float>(),
                carto::sensor::TransformTimedPointCloud(
                        ranges, sensor_to_tracking->cast<float>())});
    }
}
