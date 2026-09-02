#ifndef XIONGSIONG_DRIVER_H
#define XIONGSIONG_DRIVER_H
#include "../ConnectDriver/ConnectDriver.h"
#include "../ConnectDriver/NetworkDriver.h"
#include <boost/shared_ptr.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "connection_address.h"
#include "protoc.h"
#include <vector>
#include <deque>
#include "laserprotocol.h"
#include <mutex>
#include "hi_ros2/msg/laser_info.hpp"
#include "hi_ros2/msg/area_com.hpp"
#include "hi_ros2/srv/cmd_srv.hpp"
#include <iomanip>
#include "shadowsfilter.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
using namespace std;

using Cmdsrc = hi_ros2::srv::CmdSrv;

class XingsongDriver
{
public:
    XingsongDriver(rclcpp::Node::SharedPtr node);
    void Init();

    void Start();

private:
    boost::shared_ptr<ConnectDriver> driver_hdr;    
    boost::shared_ptr<NetworkDriver> boostdriver_hdr; 
    vector<unsigned char> databuf;
    mutable std::mutex m_data_mutex_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;

     rclcpp::Publisher<hi_ros2::msg::LaserInfo>::SharedPtr laser_info_pub;

    rclcpp::Subscription<hi_ros2::msg::AreaCom>::SharedPtr areacom_sub;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub;

     rclcpp::Service<Cmdsrc>::SharedPtr service;
   
    bool Area1, Area2,Area3;                   // 三个区域的bool值
    int NowChannel;
    bool y_direction;
    int have_block_;
    int _now_channel_;
    int faultcode = 0;

    float start_angle, end_angle;
    std::string topic_name, frame_name, service_name;

    float start_angle_rad, end_angle_rad;     // 角度过滤参数-弧度制

    ConnectionAddress laser_conn_info;
    XingSongLaserParam laser_param;
    ShadowsFilterParam shadows_filter_param;
    DisturbFilterParam disturb_filter_param;
    unsigned long long last_system_time_stamp = 0;
    short int laser_steady_time = 500; 
    bool scan_all_point_num_init_flag = true;
    double angle_increment_ = 0.0;
    uint32_t scan_all_point_num_ = 0;
    uint64_t last_data_time_;    

    void SendMsg(std::vector<unsigned char> &data);
    void SendStartCapture();
    void SendStopCapture();
    void SendSyncStartCapture();
    void SendGetVersion();
    void ResetLaser();
     bool CheckLaserParam(XingSongLaserParam& laser_param);
   void ParamInit();
     void DataReceivedCallback(std::vector<unsigned char> &data,int size);
     void InitDevice();
    int16_t FindPacketStart(vector<unsigned char> &databuf,int16_t &head_index);
    void DataHandle(vector<unsigned char> &databuf);
    bool GetAreaData(vector<unsigned char> &databuf,int head_index);
     bool GetRangeData(vector<unsigned char> &databuf,int head_index);
    bool GetSyncRangeData(vector<unsigned char> &databuf,int head_index);
    bool GetVersionData(vector<unsigned char> &databuf,int head_index);
    bool GetSeRangeData(vector<unsigned char> &databuf,int head_index);
    bool GetDeRangeData(vector<unsigned char> &databuf,int head_index);
    void SendAreaCommand();

    void DataPibilsh();
    void LaserInfoPibilsh();
    void DeDataPibilsh();

    void ntptotime(uint64_t ntptim);
    void ntp_to_ros2_stamp(uint64_t ntptim, sensor_msgs::msg::LaserScan &msg);
     sensor_msgs::msg::LaserScan ToLaserScan(const ScanData &data, float start_angle, 
        float end_angle, std::string frame_name);

     void handle_service(const std::shared_ptr<Cmdsrc::Request> request,
                      std::shared_ptr<Cmdsrc::Response> response);

    void SetAreaCallback(const hi_ros2::msg::AreaCom::SharedPtr msg);      
    
   void ScanToCloud2(
    ScanData& scan,
    sensor_msgs::msg::PointCloud2& cloud,
    bool append = false);

    int recvcount = 0;

    std::deque<ScanData> scan_data_; 
    
    bool synctype = false;
    bool block_enable = false;
    int  blockcount = 0;
    int blocktime =  5;

    float offset_angle = 0.0;


    double angle_min = 0.0;

    double angle_max = 0.0;

      HSGetAreaDataPackage command;
rclcpp::Node::SharedPtr node_;


    int32_t  last_unix_seconds = 0;

    uint32_t last_nanoseconds = 0;


    uint16_t device_type = 0;

    uint16_t device_version = 0;

    uint16_t device_SN = 0;

    int  lasercmd = 0;


    int laser_type = 1;

    int start_laser_angle = 0;
    int end_laser_angle = 360;

    double start_laser_rad = 0.0;
    double end_laser_rad = 0.0;

    double detaangle  = 0.0;

     ShadowsFilter ShadowsFilterHandle; 
     
     bool StopPointClound = false;
};

#endif // XIONGSIONG_DRIVER_H
