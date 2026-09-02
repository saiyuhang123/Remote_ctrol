#include "xingsong_driver.h"


inline uint64_t toMillis(const rclcpp::Time & t)
{
    return static_cast<uint64_t>(t.nanoseconds() / 1000000ULL);
}


XingsongDriver::XingsongDriver(rclcpp::Node::SharedPtr node)
: node_(node)
{
  //this->node_ = node_;
}

void XingsongDriver::Init()
{

    

    // 角度过滤参数
    node_->declare_parameter("start_angle", 0.0f);
    node_->get_parameter_or<float>("start_angle", start_angle, 0.0f);
    node_->declare_parameter("end_angle", 0.0f);
    node_->get_parameter_or<float>("end_angle", end_angle, 2 * M_PI);

    node_->declare_parameter("topic_name", std::string("scan1"));
    node_->get_parameter_or<std::string>("topic_name", topic_name, std::string("scan1"));

    node_->declare_parameter("frame_name", std::string("laser1"));
    node_->get_parameter_or<std::string>("frame_name", frame_name, std::string("laser1"));

   
    node_->declare_parameter("y_direction", false);
    node_->get_parameter_or<bool>("y_direction", y_direction, false);


    node_->declare_parameter("offset_angle", 0.00);
    node_->get_parameter_or<float>("offset_angle", offset_angle, 0.00);

    node_->declare_parameter("synctype", false);
    node_->get_parameter_or<bool>("synctype", synctype, false);

    node_->declare_parameter("block_enable", false);
    node_->get_parameter_or<bool>("block_enable", block_enable, false);

    node_->declare_parameter("laser_type", 1);
    node_->get_parameter_or<int>("laser_type", laser_type, 1);

    if(laser_type == 5)
    {

        scan_pub = node_->create_publisher<sensor_msgs::msg::LaserScan>("scanlaser", 3);
        point_cloud_pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, 3);
    }
    else
    {
        scan_pub = node_->create_publisher<sensor_msgs::msg::LaserScan>(topic_name, 3);
        point_cloud_pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>("scanlaser", 3);
   
    }
    



    
  
    service = node_->create_service<Cmdsrc>(
      "laser_cmd",
      std::bind(&XingsongDriver::handle_service, this,
                std::placeholders::_1, std::placeholders::_2));
       areacom_sub =  node_->create_subscription<hi_ros2::msg::AreaCom>("set_area_common", 3, 
         std::bind(&XingsongDriver::SetAreaCallback, this, std::placeholders::_1)); 


        if(laser_type == 1)    //  he
     {
        start_laser_angle = 20;
        end_laser_angle = 340;

        
        angle_min = -1 * M_PI - LasrProtocol::normalizeAngle( offset_angle / 180.0 * M_PI) + start_laser_angle / 180.0 * M_PI;
        angle_max =  M_PI - LasrProtocol::normalizeAngle( offset_angle / 180.0 * M_PI) - start_laser_angle / 180.0 * M_PI;
         
   
         
        start_laser_rad = start_laser_angle * M_PI / 180.0 + angle_min;

        start_angle_rad =  LasrProtocol::normalizeAngle(start_angle*M_PI /180.0f );


        end_angle_rad = LasrProtocol::normalizeAngle(end_angle*M_PI /180.0f );


       if(start_angle_rad < angle_min)
       {
          start_angle_rad += 2*M_PI;
       }
       if(start_angle_rad > angle_max)
       {
          start_angle_rad -= 2*M_PI;
       }

       if(end_angle_rad < angle_min)
       {
          end_angle_rad += 2*M_PI;
       }
       if(end_angle_rad > angle_max)
       {
          end_angle_rad += 2*M_PI;
       }


     }
     else if(laser_type == 2)    //fe
     {
         start_laser_angle = 0;
         end_laser_angle = 360;
        
        angle_min = -1 * M_PI - LasrProtocol::normalizeAngle( offset_angle / 180.0 * M_PI);
        angle_max =  M_PI - LasrProtocol::normalizeAngle( offset_angle / 180.0 * M_PI);

        start_laser_rad = start_laser_angle * M_PI / 180.0 + angle_min;
       
        start_angle_rad =  LasrProtocol::normalizeAngle(start_angle*M_PI /180.0f );


        end_angle_rad = LasrProtocol::normalizeAngle(end_angle*M_PI /180.0f );


      if(start_angle_rad < angle_min)
       {
          start_angle_rad += 2*M_PI;
       }
       if(start_angle_rad > angle_max)
       {
          start_angle_rad -= 2*M_PI;
       }

       if(end_angle_rad < angle_min)
       {
          end_angle_rad += 2*M_PI;
       }
       if(end_angle_rad > angle_max)
       {
          end_angle_rad += 2*M_PI;
       }


     }
     else if(laser_type == 3 || laser_type == 5)   //se
     {
        start_laser_angle = 45;
         end_laser_angle = 315;
         
        angle_min = -1 * M_PI - LasrProtocol::normalizeAngle( offset_angle / 180.0 * M_PI) + start_laser_angle / 180.0 * M_PI;
        angle_max =  M_PI - LasrProtocol::normalizeAngle( offset_angle / 180.0 * M_PI) - start_laser_angle / 180.0 * M_PI;
         

        start_laser_rad = start_laser_angle * M_PI / 180.0 + angle_min;

        start_angle_rad =  LasrProtocol::normalizeAngle(start_angle*M_PI /180.0f );


        end_angle_rad = LasrProtocol::normalizeAngle(end_angle*M_PI /180.0f );


        if(start_angle_rad < angle_min)
       {
          start_angle_rad += 2*M_PI;
       }
       if(start_angle_rad > angle_max)
       {
          start_angle_rad -= 2*M_PI;
       }

       if(end_angle_rad < angle_min)
       {
          end_angle_rad += 2*M_PI;
       }
       if(end_angle_rad > angle_max)
       {
          end_angle_rad += 2*M_PI;
       }
     }
      else if(laser_type == 4)   //le
     {
        start_laser_angle = 0;
         end_laser_angle = 360;
        

        angle_min = 0 - LasrProtocol::normalizeAngle( offset_angle / 180.0 * M_PI);
        angle_max =  2*M_PI - LasrProtocol::normalizeAngle( offset_angle / 180.0 * M_PI);

        start_laser_rad = start_laser_angle * M_PI / 180.0 + angle_min;



        start_angle_rad =  LasrProtocol::normalizeAngle(start_angle*M_PI /180.0f );


        end_angle_rad = LasrProtocol::normalizeAngle(end_angle*M_PI /180.0f );
        
       if(start_angle_rad < angle_min)
       {
          start_angle_rad += 2*M_PI;
       }
       if(start_angle_rad > angle_max)
       {
          start_angle_rad -= 2*M_PI;
       }

       if(end_angle_rad < angle_min)
       {
          end_angle_rad += 2*M_PI;
       }
       if(end_angle_rad > angle_max)
       {
          end_angle_rad += 2*M_PI;
       }
      
     }    
     else
     {
        start_laser_angle = 0;
         end_laser_angle = 360;
        
        angle_min = -1 * M_PI - LasrProtocol::normalizeAngle( offset_angle / 180.0 * M_PI);
        angle_max =  M_PI - LasrProtocol::normalizeAngle( offset_angle / 180.0 * M_PI);

        start_laser_rad = start_laser_angle * M_PI / 180.0 + angle_min;

       start_angle_rad =  LasrProtocol::normalizeAngle(start_angle*M_PI /180.0f );


        end_angle_rad = LasrProtocol::normalizeAngle(end_angle*M_PI /180.0f );
     
        if(start_angle_rad < angle_min)
       {
          start_angle_rad += 2*M_PI;
       }
       if(start_angle_rad > angle_max)
       {
          start_angle_rad -= 2*M_PI;
       }

       if(end_angle_rad < angle_min)
       {
          end_angle_rad += 2*M_PI;
       }
       if(end_angle_rad > angle_max)
       {
          end_angle_rad += 2*M_PI;
       }
           
     }

     detaangle = (end_laser_angle - start_laser_angle) / 180.0 * M_PI;

    laser_info_pub = node_->create_publisher<hi_ros2::msg::LaserInfo>("/laser_info", 10);
     
    command.angle = 0;
    command.channel = 1;
    command.channel_group = 2;      // 智能模式的通道
    command.speed = 0;
    command.mode = 1; 
  
    ParamInit();
    scan_data_.emplace_back();
    InitDevice();
}

void XingsongDriver::Start()
{
   
  rclcpp::WallRate loop_rate(12);
  while (rclcpp::ok()) 
  {


        if (!boostdriver_hdr->is_open())
        {
            RCLCPP_INFO(rclcpp::get_logger("my_logger"), "unconncet open false");
            boostdriver_hdr->Close();
            sleep(1);
         
            InitDevice();
            recvcount = 0;
        }
        else
        {

          switch(lasercmd)
          {
              case 1:         //reset
                lasercmd = 0;
                ResetLaser();
              break;

              case 2:         //startpointclound
                StopPointClound = false;
                lasercmd = 0;
                SendStartCapture();
              break;

              case 3:        //stoppointclound
                StopPointClound = true;
                lasercmd = 0;
                SendStopCapture();
              break;
               default:
                break;
          }



           int overtime =  1000 / 12 ;
           if(!StopPointClound)
           {
            if(recvcount++ > overtime ) //1s 超时
            {
                boostdriver_hdr->Close();
            }
          }


          if(block_enable)
          {
             if(blockcount++ > blocktime)
             {
                SendAreaCommand();
                blockcount = 0;
             }
             LaserInfoPibilsh();
          }


            m_data_mutex_.lock();
            DataHandle(databuf);
            m_data_mutex_.unlock();

            if(laser_type == 5)
              DeDataPibilsh();
            else
              DataPibilsh();

        }
        

        rclcpp::spin_some(node_);
        loop_rate.sleep();
    }

}


void XingsongDriver::DataPibilsh()
{
    if(scan_data_.size() < 2)
    {
        return;
    }

    ScanData data = ScanData(std::move(scan_data_.front()));
    scan_data_.pop_front();
    
     if (shadows_filter_param.max_angle < 0 || shadows_filter_param.shadows_filter_level == 0)
    {

    }
    else
      ShadowsFilterHandle.Start(data,data.distance_data.size() ,shadows_filter_param);

    sensor_msgs::msg::LaserScan scan_msg_ptr = ToLaserScan(data, start_angle_rad, end_angle_rad, frame_name);

    if(scan_msg_ptr.ranges.size() != 0)            // 如果数据长度有误，则不发布
    {  
        scan_pub->publish(scan_msg_ptr);
    }

}

sensor_msgs::msg::LaserScan XingsongDriver::ToLaserScan(const ScanData &data, float start_angle, float end_angle, std::string frame_name)
{

 sensor_msgs::msg::LaserScan msg;
  // ros帧头
  msg.header.frame_id = frame_name;

  msg.range_min = 0.05;
  msg.range_max = 35.0;

 
  msg.angle_min = angle_min;
  msg.angle_max = angle_max ;


  // 系统时间
 
  unsigned long long system_time_stamp;
  system_time_stamp = static_cast<uint64_t>(node_->get_clock()->now().nanoseconds());

  if(synctype)
  {
      
    ntptotime(data.NTPTime);
  ntp_to_ros2_stamp(data.NTPTime,msg);

     system_time_stamp = (unsigned long long)msg.header.stamp.sec*1000000000ll + (unsigned long long)msg.header.stamp.nanosec;
  }
  else
  {
      double delta_sec = (system_time_stamp - last_system_time_stamp) *
                (end_laser_angle - start_laser_angle)*1.e-9/360.0f;
      auto duration_ns = std::chrono::nanoseconds(static_cast<int64_t>(delta_sec * 1e9));
      msg.header.stamp = node_->now() - rclcpp::Duration(duration_ns);
  }
 


  // 雷达测量一周所需的时间/测量点数  

   double scantime = static_cast<float>(system_time_stamp - last_system_time_stamp)*1.e-9*
          (end_laser_angle - start_laser_angle)/ 360.0f ;

  msg.time_increment = scantime / (static_cast<float>(data.distance_data.size() - 1));

  msg.scan_time = scantime;

  last_system_time_stamp = system_time_stamp;

  int size = data.distance_data.size();


  msg.angle_increment = detaangle / float(size);
  
  msg.ranges.resize(size);
  msg.intensities.resize(size);



  for( int i = 0; i < size; i++ )
  {
     float angle = start_laser_rad + i * msg.angle_increment;
    float dis = float(data.distance_data[i])/1000.0f;

     if(start_angle <  end_angle)
    {
      if(angle >= start_angle && angle <= end_angle)
          msg.ranges[i]  = dis;
        else
          msg.ranges[i] = 1024;
    }
    else
    {
       if(angle >= start_angle || angle <= end_angle)
          msg.ranges[i]  = dis;
        else
          msg.ranges[i] = 1024.0;
    }


    msg.intensities[i] = data.amplitude_data[i];
    
  }
  return msg;
}

void XingsongDriver::ParamInit()
{
  std::string server_address, angle_increment,measure_frequency_kHz;
  int port,shadows_filter_neighbors, shadows_filter_window, shadows_filter_level, shadows_traverse_step, noise_filter_level,spin_frequency;
  float   shadows_filter_max_angle, shadows_filter_min_angle;
  bool change_flag, disturb_filter_enable, use_udp;
  int disturb_filter_threshold, disturb_filter_point_num,sampling_size_per_position;
  
  // 雷达连接参数

  node_->declare_parameter("laser_ip", std::string("192.168.1.88"));
  node_->get_parameter_or<std::string>("laser_ip", server_address, std::string("192.168.1.88"));
 
  node_->declare_parameter("laser_port", 8080);
  node_->get_parameter_or<int>("laser_port", port, 8080);

  node_->declare_parameter("use_udp", false);
  node_->get_parameter_or<bool>("use_udp", use_udp, false);


  // 雷达扫描参数
  node_->declare_parameter("spin_frequency_Hz", 25);
  node_->get_parameter_or<int>("spin_frequency_Hz", spin_frequency, 25);

  node_->declare_parameter("angle_increment", std::string("-1"));
  node_->get_parameter_or<std::string>("angle_increment", angle_increment, std::string("-1"));

  node_->declare_parameter("noise_filter_level", -1);
  node_->get_parameter_or<int>("noise_filter_level", noise_filter_level, -1);

  node_->declare_parameter("change_param", false);
  node_->get_parameter_or<bool>("change_param", change_flag, false);

  node_->declare_parameter("measure_frequency_kHz", std::string("200"));
  node_->get_parameter_or<std::string>("measure_frequency_kHz", measure_frequency_kHz, std::string("200"));

  node_->declare_parameter("sampling_size_per_position", 1);
  node_->get_parameter_or<int>("sampling_size_per_position", sampling_size_per_position, -1);


  //防拖尾参数
  node_->declare_parameter("shadows_filter_max_angle", 175.0);
  node_->get_parameter_or<float>("shadows_filter_max_angle", shadows_filter_max_angle, 175.0);

  node_->declare_parameter("shadows_filter_min_angle", 5.0);
  node_->get_parameter_or<float>("shadows_filter_min_angle", shadows_filter_min_angle, 5.0);

  node_->declare_parameter("shadows_filter_neighbors", 1);
  node_->get_parameter_or<int>("shadows_filter_neighbors", shadows_filter_neighbors, 1);

  node_->declare_parameter("shadows_filter_window", 2);
  node_->get_parameter_or<int>("shadows_filter_window", shadows_filter_window, 2);

  node_->declare_parameter("shadows_traverse_step", 1);
  node_->get_parameter_or<int>("shadows_traverse_step", shadows_traverse_step, 1);

  node_->declare_parameter("shadows_filter_level", -1);
  node_->get_parameter_or<int>("shadows_filter_level", shadows_filter_level, -1);


  // 串扰过滤参数
  node_->declare_parameter("disturb_filter_threshold", 80);
  node_->get_parameter_or<int>("disturb_filter_threshold", disturb_filter_threshold, 80);

  node_->declare_parameter("disturb_filter_point_num", 8);
  node_->get_parameter_or<int>("disturb_filter_point_num", disturb_filter_point_num, 8);

  node_->declare_parameter("disturb_filter_enable", false);
  node_->get_parameter_or<bool>("disturb_filter_enable", disturb_filter_enable, false);


  // 雷达连接参数
  laser_conn_info.SetAddress(server_address);
  laser_conn_info.SetPort(port);
  if(use_udp)
    laser_conn_info.UseUdp();   //// udp
  // 雷达扫描参数
  laser_param.change_flag = change_flag;
  laser_param.spin_frequency_Hz = spin_frequency;
  laser_param.angle_increment = angle_increment;
  laser_param.noise_filter_level = noise_filter_level;
  laser_param.measure_frequency_kHz = measure_frequency_kHz;
  laser_param.sampling_size_per_position = sampling_size_per_position;

  // 防拖尾参数
  shadows_filter_param.max_angle = shadows_filter_max_angle;
  shadows_filter_param.min_angle = shadows_filter_min_angle;
  shadows_filter_param.neighbors = shadows_filter_neighbors;
  shadows_filter_param.window = shadows_filter_window;
  shadows_filter_param.traverse_step = shadows_traverse_step;
  shadows_filter_param.shadows_filter_level = shadows_filter_level;

  // 串扰过滤参数
  disturb_filter_param.threshold = disturb_filter_threshold;
  disturb_filter_param.range = disturb_filter_point_num;
  disturb_filter_param.disturb_filter_enable = disturb_filter_enable;
}

void XingsongDriver::InitDevice()
{
        boostdriver_hdr.reset(new NetworkDriver(laser_conn_info.GetUseUdp()));

         auto binding = std::bind(&XingsongDriver::DataReceivedCallback, this, std::placeholders::_1,std::placeholders::_2);
        boostdriver_hdr->setCallback(binding);

        boostdriver_hdr->StartThreads(2);

        int setupcout = 0;
        if (boostdriver_hdr->setup(laser_conn_info.GetAddress(), laser_conn_info.GetPort()))
        {
          if(laser_conn_info.GetUseUdp())
          {
              RCLCPP_INFO(rclcpp::get_logger("my_logger"), "Laser Udp Connect success");
          }
          else
          {
            RCLCPP_INFO(rclcpp::get_logger("my_logger"), "Laser Tcp Connect success");
          }
          if(laser_param.change_flag)
          {
              std::vector<unsigned char> senddata(12,0);

              LasrProtocol::GenerateParamCommand(laser_param,senddata,laser_type);

              SendMsg(senddata);
              usleep(100000);
          }

          SendGetVersion();

          if(synctype)
            SendSyncStartCapture();
          else
            SendStartCapture();
            
        }
        else
        {
            boostdriver_hdr->Close();
            if(laser_conn_info.GetUseUdp())
            {
              RCLCPP_INFO(rclcpp::get_logger("my_logger"), "Laser Udp Connect false");
            }
            else
            {
              RCLCPP_INFO(rclcpp::get_logger("my_logger"), "Laser Tcp Connect false");
            }
        }
}

void XingsongDriver::DataReceivedCallback(std::vector<unsigned char> &data,int size)
{
    recvcount = 0;
     m_data_mutex_.lock();
    for(int i = 0; i < size; i++)
    {
        databuf.push_back(data[i]);
    }

     m_data_mutex_.unlock();
}

bool XingsongDriver::CheckLaserParam(XingSongLaserParam& laser_param)
{
  if(!laser_param.change_flag) // 如果change_flag为false，则返回false
    return false;
  if(laser_param.spin_frequency_Hz == 10 || laser_param.spin_frequency_Hz == 20)
    if(laser_param.noise_filter_level >= 0 && laser_param.noise_filter_level <= 3)
      if(laser_param.angle_increment == "0.025" || laser_param.angle_increment == "0.050"
          || laser_param.angle_increment == "0.100" || laser_param.angle_increment == "0.250"
          || laser_param.angle_increment == "0.500")
      {
        laser_param.change_flag = true;
        return true;
      }
  laser_param.change_flag = false;
  return false;
}


void XingsongDriver::SendMsg(std::vector<unsigned char> &data)
{

    unsigned char *send_data = &data[0];

    try
    {
        if (boostdriver_hdr->is_open())
        {
            bool flag = boostdriver_hdr->Send((char *)send_data, data.size());

            if(!flag)
            {
              
                boostdriver_hdr->Close();
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}

void XingsongDriver::SendStartCapture()
{
    std::vector<unsigned char> senddata(8,0);

    senddata[0] = 0x52;

    senddata[1] = 0x41;

    senddata[2] = 0x75;

    senddata[3] = 0x74;

    senddata[4] = 0x6f;

    senddata[5] = 0x01;

    senddata[6] = 0x87;

    senddata[7] = 0x80;

    SendMsg(senddata);


}

void XingsongDriver::SendSyncStartCapture()
{
    std::vector<unsigned char> senddata(8,0);

    senddata[0] = 0x52;

    senddata[1] = 0x41;

    senddata[2] = 0x75;

    senddata[3] = 0x74;

    senddata[4] = 0x6f;

    senddata[5] = 0x02;

    senddata[6] = 0xC7;

    senddata[7] = 0x81;

    SendMsg(senddata);


}

void XingsongDriver::SendGetVersion()
{
    std::vector<unsigned char> senddata(7,0);

    senddata[0] = 0x52;

    senddata[1] = 0x50;

    senddata[2] = 0x72;

    senddata[3] = 0x64;

    senddata[4] = 0x61;

    senddata[5] = 0xc7;

    senddata[6] = 0xff;

    SendMsg(senddata);


}

 void XingsongDriver::ResetLaser()
 {
    std::vector<unsigned char> senddata(7,0);

    senddata[0] = 0x52;

    senddata[1] = 0x42;

    senddata[2] = 0x6f;

    senddata[3] = 0x6f;

    senddata[4] = 0x74;

    senddata[5] = 0x94;

    senddata[6] = 0x7e;

    SendMsg(senddata);

 }

// // 根据协议寻找数据帧头
int16_t XingsongDriver::FindPacketStart(vector<unsigned char> &databuf,int16_t &head_index)
{

  
  
  if(synctype)
  {
    if(databuf.size() < SynckXSPackageHeadSize)
    return -1;
  }
  else
  {
    if(databuf.size() < kXSPackageHeadSize)
    return -1;
  }


  // 兴颂雷达数据的帧头
  for(size_t i = 0; i < databuf.size() - 4; i++)
  {
    if(0x48 == ((unsigned char)databuf[i])   &&
       0x49 == ((unsigned char)databuf[i+1]) &&
       0x53 == ((unsigned char)databuf[i+2]) &&
       0x4e == ((unsigned char)databuf[i+3]))
     {
     // area_data_flag_ = false;
      head_index = i;
      return 1;   //GetRangeData 
    }
    else if
    (
      0x57 == ((unsigned char)databuf[i])   &&
       0x53 == ((unsigned char)databuf[i+1]) &&
       0x69 == ((unsigned char)databuf[i+2]) &&
       0x6d == ((unsigned char)databuf[i+3]) &&
       0x75 == ((unsigned char)databuf[i+4])
    )
    {
      head_index = i;

      return 2;   // GetAreaData
    }
    else if
    (
      'R' == ((unsigned char)databuf[i])   &&
      'P' == ((unsigned char)databuf[i+1]) &&
      'r' == ((unsigned char)databuf[i+2]) &&
      'd' == ((unsigned char)databuf[i+3]) &&
      'a' == ((unsigned char)databuf[i+4])
    )
    {
      head_index = i;

      return 3;   // 
    }
    else if
    (
      0x48 == ((unsigned char)databuf[i])   &&
       0x49 == ((unsigned char)databuf[i+1]) &&
       0x4b == ((unsigned char)databuf[i+2]) &&
       0x41 == ((unsigned char)databuf[i+3]) 
    )
    {
      head_index = i;

      return 4;   // GetsyncData
    }
    else if(
      0x43 == ((unsigned char)databuf[i])   &&
       0x6c == ((unsigned char)databuf[i+1]) &&
       0x6f == ((unsigned char)databuf[i+2]) &&
       0x75 == ((unsigned char)databuf[i+3]) &&
       0x64 == ((unsigned char)databuf[i+4])
    )
    {
      head_index = i;
      return 5;   // GetsyncData
    }
  }
  return -2;
}


void XingsongDriver::DataHandle(vector<unsigned char> &databuf)
{
  
  int16_t head_index  = 0;                     // 寻找帧头

  int16_t code = 0;
  while(code >= 0)
  {
    
    code = FindPacketStart(databuf,head_index);
    bool flag = false;
    if(code == 2)
    {
      flag = GetAreaData(databuf,head_index);
    }
    else if(code == 1)
    {
       flag = GetRangeData(databuf,head_index);
    }
    else if(code == 3)
    {
       flag = GetVersionData(databuf,head_index);
    }
    else if(code == 4)
    {
      flag = GetSyncRangeData(databuf,head_index);
    }
    else if(code == 5)
    {
      if(laser_type == 3)
        flag = GetSeRangeData(databuf,head_index);
        else if(laser_type == 5)    //de
        {
            flag = GetDeRangeData(databuf,head_index);
        }
    }



    if(!flag)
    {
        return;
    }
  }


}

bool XingsongDriver::GetAreaData(vector<unsigned char> &databuf,int head_index)
{
  bool ret = false;
  // 寻找帧头并处理数据
    if(databuf.size() - head_index >= kHSAreaDataPackageSize)
    {
        have_block_ = int(databuf[head_index + 7]);
        _now_channel_ = int(databuf[head_index + 5]);

        faultcode = (unsigned char)databuf[head_index + 9] << 8;     
        faultcode |= (unsigned char)databuf[head_index + 10];
        ret = true;
       databuf.erase(databuf.begin(),databuf.begin() + head_index + kHSAreaDataPackageSize);
       return ret;
    }
    return ret;    
}

void XingsongDriver::SendAreaCommand()
{

  std::vector<unsigned char> senddata(18,0);

  LasrProtocol::SetAreaCommand(command,senddata);

   SendMsg(senddata);
}

bool XingsongDriver::GetRangeData(vector<unsigned char> &databuf,int head_index)
{
    bool ret = false;
    if(databuf.size() - head_index >= kXSPackageHeadSize)
    {
        vector<unsigned char> head_buf;

        head_buf.insert(head_buf.end(),databuf.begin() + head_index, databuf.begin() + head_index + kXSPackageHeadSize);

        XSPackageHeader header;
        header.start_angle = (unsigned char)head_buf[4] << 8;     // 起始角度
        header.start_angle |= (unsigned char)head_buf[5];
        header.end_angle = (unsigned char)head_buf[6] << 8;       // 终止角度
        header.end_angle |= (unsigned char)head_buf[7];
        header.data_size = (unsigned char)head_buf[8] << 8;       // 测量点总数
        header.data_size |= (unsigned char)head_buf[9];
        header.data_position = (unsigned char)head_buf[10] << 8;  // 当前帧测量点位置
        header.data_position |= (unsigned char)head_buf[11];
        header.measure_size = (unsigned char)head_buf[12] << 8;   // 当前帧测量点数量
        header.measure_size |= (unsigned char)head_buf[13];
        header.time = (unsigned char)head_buf[14] << 8;           // 当前帧测量时间戳
        header.time |= (unsigned char)head_buf[15];
      
        // << std::endl;

         int endangle = header.end_angle ;

        if(endangle == 0)
          endangle = 360;



        if(header.data_size > header.measure_size)
        {
            header.data_size = header.measure_size;
        }

        if((header.start_angle==start_laser_angle && laser_steady_time > 0) || scan_all_point_num_init_flag)         // 前几帧数据异常
        {
            // 计算角度分辨率
            angle_increment_ = double(endangle - header.start_angle)/header.measure_size;
            scan_all_point_num_ =(end_laser_angle - start_laser_angle)/angle_increment_;
            scan_all_point_num_init_flag = false;
            laser_steady_time--;     
        }

        // 2. 解析帧内容
        ScanData& scan_data = scan_data_.back();
        scan_data.distance_data.resize(scan_all_point_num_);
        scan_data.amplitude_data.resize(scan_all_point_num_);

        uint16_t body_size = kXSPackageHeadSize + header.data_size * 4;
        if((databuf.size() - head_index) >= body_size)
        {
            ret = true;
            vector<unsigned char> body_buf;
            body_buf.insert(body_buf.end(),databuf.begin() + head_index, databuf.begin() + head_index + body_size);
            databuf.erase(databuf.begin(),databuf.begin() + head_index + body_size);
            unsigned int begin_point_index = 0;

            // 计算本帧第一点的索引值
            begin_point_index = (header.start_angle - start_laser_angle)/angle_increment_+header.data_position-header.data_size;
            if(scan_all_point_num_ < begin_point_index+header.data_size)
            {
            scan_data.distance_data.resize(begin_point_index+header.data_size);
            scan_data.amplitude_data.resize(begin_point_index+header.data_size);
            }
            for(int i = 0; i < header.data_size; i++)
            {
                unsigned short int distance;
                unsigned short int intensity;
                distance = (unsigned char)body_buf[i*4 + 17] * 256;
                distance |= (unsigned char)body_buf[i*4 + 16];
                intensity = (unsigned char)body_buf[i*4 + 19] * 256;
                intensity |= (unsigned char)body_buf[i*4 + 18];

                if(distance > kMaxDistance)
                {
                    distance = kMaxDistance+10000;
                }
                if(intensity > kMaxIntensity)
                {
                    intensity = kMaxIntensity;
                }
                try {
                    scan_data.distance_data.at(begin_point_index+i) = distance;
                    scan_data.amplitude_data.at(begin_point_index+i) = intensity;
                } catch (std::exception& e) 
                {
                     cout << "distance_data error" << endl;
                } 
            }
            uint64_t now_time = toMillis(node_->now());
            if ((now_time - last_data_time_) > 1000) // 与上帧时间超过1s
            {

            }


           

            if(header.start_angle == start_laser_angle)                             // 第一帧的时间戳存到scan_data.time_increment里
                scan_data.time_increment=float(header.time);

            // 接收完一圈数据
            else if(endangle == end_laser_angle && header.data_position == header.measure_size)
            {
                if(header.time - scan_data.time_increment < 0)        // 将一圈的时间放进scan_data.time_increment里
                scan_data.time_increment = 65535 - scan_data.time_increment + header.time;
                else
                scan_data.time_increment = header.time - scan_data.time_increment;

                scan_data_.emplace_back();
                if(scan_data_.size() > 5)
                {
                scan_data_.pop_front();
                }
                last_data_time_ = toMillis(node_->now());
            }

        }
    
      }

    return ret;
}

bool XingsongDriver::GetSyncRangeData(vector<unsigned char> &databuf,int head_index)
{
  bool ret = false;
      // 寻找帧头并处理数据

    if(databuf.size() - head_index >= SynckXSPackageHeadSize)
    {
      vector<unsigned char> head_buf;

      head_buf.insert(head_buf.end(),databuf.begin() + head_index, databuf.begin() + head_index + SynckXSPackageHeadSize);


    SyncXSPackageHeader header;
    header.start_angle = (unsigned char)head_buf[4] << 8;     // 起始角度
    header.start_angle |= (unsigned char)head_buf[5];
    header.end_angle = (unsigned char)head_buf[6] << 8;       // 终止角度
    header.end_angle |= (unsigned char)head_buf[7];
    header.data_size = (unsigned char)head_buf[8] << 8;       // 测量点总数
    header.data_size |= (unsigned char)head_buf[9];
    header.data_position = (unsigned char)head_buf[10] << 8;  // 当前帧测量点位置
    header.data_position |= (unsigned char)head_buf[11];
    header.measure_size = (unsigned char)head_buf[12] << 8;   // 当前帧测量点数量
    header.measure_size |= (unsigned char)head_buf[13];
    header.time = (unsigned char)head_buf[14] << 8;   
    header.time |= (unsigned char)head_buf[15];


    header.timems = (unsigned char)head_buf[16] << 24;
    header.timems |= (unsigned char)head_buf[17] << 16;
    header.timems |= (unsigned char)head_buf[18] << 8;   
    header.timems |= (unsigned char)head_buf[19];


    uint32_t NTPTime1 , NTPTime2;
    uint64_t NTPTime3;

    NTPTime1 = (unsigned char)head_buf[20] << 24;
    NTPTime1 |= (unsigned char)head_buf[21] << 16;
    NTPTime1 |= (unsigned char)head_buf[22] << 8;   
    NTPTime1 |= (unsigned char)head_buf[23];  

    NTPTime2 = (unsigned char)head_buf[24] << 24;
    NTPTime2 |= (unsigned char)head_buf[25] << 16;
    NTPTime2 |= (unsigned char)head_buf[26] << 8;   
    NTPTime2 |= (unsigned char)head_buf[27];  


    NTPTime3 = ((uint64_t)NTPTime1 << 32) | NTPTime2;

    if(header.data_size > header.measure_size)
    {
      header.data_size = header.measure_size;
    }
      // 计算角度分辨率
    angle_increment_ = double(header.end_angle - header.start_angle)/header.measure_size;
    scan_all_point_num_ = (end_laser_angle - start_laser_angle)/angle_increment_;
    scan_all_point_num_init_flag = false;
      // rec_begin_flag_ += 1;
    if(laser_steady_time>=0)
      laser_steady_time--;


    // 2. 解析帧内容
    ScanData& scan_data = scan_data_.back();
    scan_data.distance_data.resize(scan_all_point_num_);
    scan_data.amplitude_data.resize(scan_all_point_num_);
    
    uint16_t body_size = SynckXSPackageHeadSize + header.data_size * 4;
    if((databuf.size() - head_index) >= body_size)
    {
      ret = true;
      vector<unsigned char> body_buf;
      body_buf.insert(body_buf.end(),databuf.begin() + head_index, databuf.begin() + head_index + body_size);
      databuf.erase(databuf.begin(),databuf.begin() + head_index + body_size);
      unsigned int begin_point_index = 0;

      // 计算本帧第一点的索引值




      begin_point_index = (header.start_angle-start_laser_angle)/angle_increment_+header.data_position-header.data_size;
     
      if(scan_all_point_num_ < begin_point_index+header.data_size)
      {
        scan_data.distance_data.resize(begin_point_index+header.data_size);
        scan_data.amplitude_data.resize(begin_point_index+header.data_size);
      }

      #ifdef DEBUG
      std::cout << "begin:" << header.start_angle 
                << "  end:" << header.end_angle 
                << "  data_size:" << header.data_size
                << "  data_position:" << header.data_position
                << "  measure_size:" << header.measure_size
                << "  time:" << header.time
                << "  laser_steady_time:" << laser_steady_time
                << "  begin_point_index:" << begin_point_index
                << "  scan_all_point_num_:" << scan_all_point_num_
                << std::endl;
      #endif
      for(int i = 0; i < header.data_size; i++)
      {

       // cout << "distance" << endl;
        unsigned short int distance;
        unsigned short int intensity;
        distance = (unsigned char)body_buf[i*4 + 29] * 256;
        distance |= (unsigned char)body_buf[i*4 + 28];
        intensity = (unsigned char)body_buf[i*4 + 31] * 256;
        intensity |= (unsigned char)body_buf[i*4 + 30];
          
        if(distance > kMaxDistance)
        {
          distance = kMaxDistance+10000;
        }
        if(intensity > kMaxIntensity)
        {
          intensity = kMaxIntensity;
        }
        try {
          scan_data.distance_data.at(begin_point_index+i) = distance;
          scan_data.amplitude_data.at(begin_point_index+i) = intensity;
        } catch (std::exception& e) {
          cout << e.what() << endl;
        } 
      }

      uint64_t now_time = toMillis(node_->now());
      if ((now_time - last_data_time_) > 1000) // 与上帧时间超过1s
      {
        last_data_time_ = now_time;
       }

      if(header.start_angle == start_laser_angle)     
      {
         scan_data.time_increment=float(header.time);

      }                       
      // 接收完一圈数据
      else if(header.end_angle == end_laser_angle && header.data_position == header.measure_size)
      {

        if(header.time - scan_data.time_increment < 0)        // 将一圈的时间放进scan_data.time_increment里
          scan_data.time_increment = 65535 - scan_data.time_increment + header.time;
        else
          scan_data.time_increment = header.time - scan_data.time_increment;

        scan_data.NTPTime = NTPTime3;

        scan_data_.emplace_back();
        if(scan_data_.size() > 5)
        {
          scan_data_.pop_front();
        }
        last_data_time_ = toMillis(node_->now());
      }
    
    }
  }
  
  return ret;
  
}



bool XingsongDriver::GetSeRangeData(vector<unsigned char> &databuf,int head_index)
{

    bool ret = false;
    // 寻找帧头并处理数据
    if(databuf.size() - head_index >= kSEPackageHeadSize)
    {
        vector<unsigned char> head_buf;

        head_buf.insert(head_buf.end(),databuf.begin() + head_index, databuf.begin() + head_index + kSEPackageHeadSize);
        // 1. 解析数据帧帧头

        SEPackageHeader header;
        header.start_angle = (unsigned char)head_buf[6] << 8;     // 起始角度
        header.start_angle |= (unsigned char)head_buf[7];
        header.end_angle = (unsigned char)head_buf[8] << 8;       // 终止角度
        header.end_angle |= (unsigned char)head_buf[9];
        header.data_size = (unsigned char)head_buf[10] << 8;       // 测量点总数
        header.data_size |= (unsigned char)head_buf[11];

        // 2. 解析帧内容
        ScanData& scan_data = scan_data_.back();


        uint16_t body_size = kSEPackageHeadSize + header.data_size * 4;

        if((databuf.size() - head_index) >= body_size)
        {
            ret = true;
            vector<unsigned char> body_buf;
            body_buf.insert(body_buf.end(),databuf.begin() + head_index, databuf.begin() + head_index + body_size);

            databuf.erase(databuf.begin(),databuf.begin() + head_index + body_size);
            unsigned int begin_point_index = 0;

            // 计算本帧第一点的索引值
            begin_point_index = (header.start_angle - start_laser_angle)/((header.end_angle-header.start_angle)/float(header.data_size));
            scan_data.distance_data.resize(begin_point_index+header.data_size);
            scan_data.amplitude_data.resize(begin_point_index+header.data_size);

            for(int i = 0; i < header.data_size; i++)
            {
                unsigned short int distance;
                unsigned short int intensity;
                distance = (unsigned char)body_buf[i*4 + 13] * 256;
                distance |= (unsigned char)body_buf[i*4 + 12];
                intensity = (unsigned char)body_buf[i*4 + 15] * 256;
                intensity |= (unsigned char)body_buf[i*4 + 14];

                if(distance > kMaxDistance)
                {
                    distance = kMaxDistance+10000;
                }
                if(intensity > kMaxIntensity)
                {
                    intensity = kMaxIntensity;
                }
                try {
                    scan_data.distance_data.at(begin_point_index+i) = distance;
                    scan_data.amplitude_data.at(begin_point_index+i) = intensity;
                } catch (std::exception& e) 
                {
                     cout << "distance_data error" << endl;
                } 
            }


             uint64_t now_time = toMillis(node_->now());

   
            // 接收完一圈数据

            if(header.end_angle == end_laser_angle )
            {
               
                scan_data_.emplace_back();
                if(scan_data_.size() > 5)
                {
                scan_data_.pop_front();
                }
                last_data_time_ = toMillis(node_->now());
            }

        }
    
      }

    return ret;
}


void XingsongDriver::LaserInfoPibilsh()
{
    auto msg = hi_ros2::msg::LaserInfo();
    msg.laserstate = faultcode;
    msg.area1 = have_block_&0x01;
    msg.area2 = have_block_&0x02;
    msg.area3 = have_block_&0x04;
    msg.channel = _now_channel_;
    msg.devicecode = device_type;
    msg.deviceversion = device_version;
    msg.devicesn = device_SN;
  laser_info_pub->publish(msg);
}

void XingsongDriver::ntptotime(uint64_t ntptim)
{
    // 使用标准定义的纪元差值（更准确）
    const uint32_t ntp_to_unix_epoch = 2208988800U; // 这个值在RFC中明确定义

    uint64_t ntp_timestamp = ntptim;
    uint32_t ntp_seconds = (ntp_timestamp >> 32) & 0xFFFFFFFF;
    uint32_t ntp_fraction = ntp_timestamp & 0xFFFFFFFF;

    // 检查是否溢出（处理2036年问题）
    if (ntp_seconds < ntp_to_unix_epoch) {
     //   std::cerr << "Error: NTP time before Unix epoch" << std::endl;
        return;
        
    }

    time_t unix_time = static_cast<time_t>(ntp_seconds - ntp_to_unix_epoch);

    // 使用线程安全的gmtime_r（如果可用）
    struct tm utc_time_struct;
#ifdef _WIN32
    gmtime_s(&utc_time_struct, &unix_time);
#else
    gmtime_r(&unix_time, &utc_time_struct);
#endif

  //  std::cout << "UTC time: " << std::put_time(&utc_time_struct, "%Y-%m-%d %H:%M:%S");

    // 更精确的分数秒计算
    double fraction_sec = static_cast<double>(ntp_fraction) / 4294967296.0; // 2^32
    // std::cout << "." << std::fixed << std::setprecision(9)
    //     << fraction_sec << std::endl;

   
}

void XingsongDriver::ntp_to_ros2_stamp(uint64_t ntptim, sensor_msgs::msg::LaserScan &msg)
{
    const uint32_t ntp_to_unix_epoch = 2208988800U;

    uint32_t ntp_seconds  = (ntptim >> 32) & 0xFFFFFFFF;
    uint32_t ntp_fraction = ntptim & 0xFFFFFFFF;

    // 转成 Unix 秒
    if (ntp_seconds < ntp_to_unix_epoch) {
       // throw std::runtime_error("Invalid NTP time (before Unix epoch)");
        return ;
    }
    uint32_t unix_seconds = ntp_seconds - ntp_to_unix_epoch;

    // 转成纳秒
    uint32_t nanoseconds = static_cast<uint32_t>(
        (static_cast<uint64_t>(ntp_fraction) * 1000000000ULL) >> 32
    );

    // 填充 LaserScan 的时间戳
    msg.header.stamp.sec = last_unix_seconds;
    msg.header.stamp.nanosec = last_nanoseconds;


    last_unix_seconds = static_cast<int32_t>(unix_seconds);

    last_nanoseconds = nanoseconds;

}

 bool XingsongDriver::GetVersionData(vector<unsigned char> &databuf,int head_index)
 {
    bool ret = false;
  // 寻找帧头并处理数据
    if(databuf.size() - head_index >= 561)
    {

        // 获取帧


      device_type = (unsigned char)databuf[head_index + 5] << 8;     
      device_type |= (unsigned char)databuf[head_index + 6];
      

      device_version = (unsigned char)databuf[head_index + 7] << 8;     
      device_version |= (unsigned char)databuf[head_index + 8];

      device_SN = (unsigned char)databuf[head_index + 9] << 8;     
      device_SN |= (unsigned char)databuf[head_index + 10];
        ret = true;
       databuf.erase(databuf.begin(),databuf.begin() + head_index + 561);
       return ret;
    }
    return ret;    
 }

void XingsongDriver::handle_service(const std::shared_ptr<Cmdsrc::Request> request,
                      std::shared_ptr<Cmdsrc::Response> response)
{
  
    lasercmd = request->cmd;
cout << "lasercmd:" << lasercmd << endl;
  response->success = true;
  
}

void XingsongDriver::SetAreaCallback(const hi_ros2::msg::AreaCom::SharedPtr msg)
{
    command.angle = msg->angle;
    command.channel = msg->channel;
    command.channel_group = msg->channel_group;
    command.speed = msg->speed;
    command.mode = msg->mode;
}

void XingsongDriver::SendStopCapture()
{
    std::vector<unsigned char> senddata(8,0);
  //  kStartCapture {0x52, 0x41, 0x75, 0x74, 0x6f, 0x01, 0x87, 0x80};

    senddata[0] = 0x52;

    senddata[1] = 0x41;

    senddata[2] = 0x75;

    senddata[3] = 0x74;

    senddata[4] = 0x6f;

    senddata[5] = 0x00;

    senddata[6] = 0x46;

    senddata[7] = 0x40;

   cout << "SendStopCapture" << endl;
  // sleep(5);

    SendMsg(senddata);


}

bool XingsongDriver::GetDeRangeData(vector<unsigned char> &databuf,int head_index)
{

    bool ret = false;
    // 寻找帧头并处理数据
  //  cout << "databufsize0:" << databuf.size()  << endl;
    if(databuf.size() - head_index >= kDEPackageHeadSize)
    {
        vector<unsigned char> head_buf;

        head_buf.insert(head_buf.end(),databuf.begin() + head_index, databuf.begin() + head_index + kDEPackageHeadSize);
        // 1. 解析数据帧帧头

        DEPackageHeader header;
        header.line_index = head_buf[5];
        header.start_angle = (unsigned char)head_buf[6] << 8;     // 起始角度
        header.start_angle |= (unsigned char)head_buf[7];
        header.end_angle = (unsigned char)head_buf[8] << 8;       // 终止角度
        header.end_angle |= (unsigned char)head_buf[9];
        header.data_size = (unsigned char)head_buf[10] << 8;       // 测量点总数
        header.data_size |= (unsigned char)head_buf[11];


        // 2. 解析帧内容
        ScanData& scan_data = scan_data_.back();

        uint16_t body_size = kDEPackageHeadSize + header.data_size * 4;
        if((databuf.size() - head_index) >= body_size)
        {
            ret = true;
            vector<unsigned char> body_buf;
            body_buf.insert(body_buf.end(),databuf.begin() + head_index, databuf.begin() + head_index + body_size);

            databuf.erase(databuf.begin(),databuf.begin() + head_index + body_size);
            
            for(int i = 0; i < header.data_size; i++)
            {
                unsigned short int distance;
                unsigned short int intensity;
                distance = (unsigned char)body_buf[i*4 + 13] * 256;
                distance |= (unsigned char)body_buf[i*4 + 12];
                intensity = (unsigned char)body_buf[i*4 + 15] * 256;
                intensity |= (unsigned char)body_buf[i*4 + 14];

                if(distance > kMaxDistance)
                {
                    distance = kMaxDistance+10000;
                }
                if(intensity > kMaxIntensity)
                {
                    intensity = kMaxIntensity;
                }

                scan_data.distance_data.push_back(distance);
                scan_data.amplitude_data.push_back(intensity);
               
            }

            scan_data.line_index = header.line_index;
          if(scan_data.line_index == 4)
              scan_data.line_index = 3;
          else if(scan_data.line_index == 3)
              scan_data.line_index = 4;
            uint64_t now_time = toMillis(node_->now() );

   
            // 接收完一圈数据


            if(header.end_angle == 240 )
            {
                scan_data_.emplace_back();
                if(scan_data_.size() > 9)
                {
                scan_data_.pop_front();
                }
                last_data_time_ = toMillis(node_->now());
            }

        }
    
      }

    return ret;
}


void XingsongDriver::ScanToCloud2(
    ScanData& scan,
    sensor_msgs::msg::PointCloud2& cloud,
    bool append) // 新增参数，默认覆盖
{
    const size_t point_num = scan.distance_data.size();
    if (point_num == 0) return;

    if(!append) {
        // ------------------ 初始化 cloud ------------------
        cloud.header.frame_id = frame_name;
        cloud.height = 1;
        cloud.width = point_num;
        cloud.is_dense = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(
            4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32
        );

        modifier.resize(point_num);
    } else {
        // ------------------ 追加模式 ------------------
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        size_t old_size = cloud.width;
        cloud.width += point_num;
        modifier.resize(cloud.width);
    }

    // ------------------ 迭代器 ------------------
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_i(cloud, "intensity");

    // 移动迭代器到追加位置
    for(size_t j=0; j<cloud.width-point_num; ++j){
        ++iter_x; ++iter_y; ++iter_z; ++iter_i;
    }

    // ------------------ 填充点 ------------------
    float increment = 120.0f / point_num;
    float horizon_rad = static_cast<float>(10 - scan.line_index * 4) * M_PI / 180.f;

    for (size_t i = 0; i < point_num; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_i)
    {
        if (scan.distance_data[i] >= 4000)
        {
            *iter_x = NAN;
            *iter_y = NAN;
            *iter_z = NAN;
            *iter_i = static_cast<float>(scan.amplitude_data[i]);
            continue;
        }

        float angle = (120.0f + increment * i) * M_PI / 180.0f;
        float distance = scan.distance_data[i] / 1000.0f;

        *iter_x = distance * std::cos(angle);
        *iter_y = distance * std::sin(angle);
        *iter_z = distance * std::sin(horizon_rad);
        *iter_i = static_cast<float>(scan.amplitude_data[i]);
    }
}


void XingsongDriver::DeDataPibilsh()
{
  
  sensor_msgs::msg::PointCloud2 cloud;

  if(scan_data_.size() < 5)
  {
      return;
  }
bool first = true;
  for(int i = 0; i < 4 ; i++)
  {
      ScanData data = ScanData(std::move(scan_data_.front()));
      scan_data_.pop_front();

      ScanToCloud2(data, cloud,!first);
      first = false;
  }

  point_cloud_pub->publish(cloud);  

}