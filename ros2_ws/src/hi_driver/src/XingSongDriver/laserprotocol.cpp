#include "laserprotocol.h"

LasrProtocol::LasrProtocol()
{

}
// CRC循环校验码-12位命令
// 修改command的最后两位
// 返回循环校验码的10进制数字
unsigned int LasrProtocol::CRC_Verify_len12( vector<unsigned char> &command)
{
  unsigned int i, j;
  unsigned int wCrc = 0xffff;
  unsigned int wPolynom = 0xA001;
  /*-------------------------------------*/
  for (i = 0; i < 12 - 2; i++) // 后两位为校验位
  {
    wCrc ^= command[i];
    for (j = 0; j < 8; j++)
    {
      if (wCrc & 0x0001)
      {
        wCrc = (wCrc >> 1) ^ wPolynom;
      }
      else
      {
        wCrc = wCrc >> 1;
      }
    }
  }
  unsigned int tmp1 = (wCrc & 0xff00) >> 8;           // 高位
  unsigned int tmp2 = (wCrc & 0x00ff);                // 低位
  command[10] = tmp2;
  command[11] = tmp1;
  return wCrc;
}


// 生成雷达控制命令
void LasrProtocol::GenerateParamCommand(XingSongLaserParam param,vector<unsigned char> &data,int type)
{
  string run_state = param.run_state;

  int spin_frequency_Hz = param.spin_frequency_Hz;
  string angle_increment = param.angle_increment;
  int noise_filter_level = param.noise_filter_level;
   string measure_frequency_kHz = param.measure_frequency_kHz;
int sample_size_int = param.sampling_size_per_position;
  //data.clear();
                 // 命令初始化全为0

  //----------设置数据帧头----------//
    data[0] = 'S';
    data[1] = 'C';
    data[2] = 't';
    data[3] = 'r';
    data[4] = 'l';

  //----------设置休眠控制----------//
  if (run_state == "run")
    data[5] = 0x00;
  else if (run_state == "stop")
    data[5] = 0x01;
  else
  {
    //ROS_WARN_STREAM("run_state error!");// 若输入非法,则为run状态
  }
    
  //------------尚未配置------------//

  if(type == 4)
  {
      if (measure_frequency_kHz == "50")
        data[6] = 0x00;
    else if (measure_frequency_kHz == "100")
        data[6] = 0x01;
    else if (measure_frequency_kHz == "150")
        data[6] = 0x02;
    else if (measure_frequency_kHz == "200")
        data[6] = 0x03;
    else // 若输入非法,则为200kHz
    {
        data[6] = 0x03;
        
    }
  }
  else
   data[6] = 0x00;

  //----------设置扫描频率----------//
  if(type == 1)
  {
      if (spin_frequency_Hz == 15)
        data[7] = 0x00;
      else if (spin_frequency_Hz == 30)
        data[7] = 0x01;
      else                                               // 若输入非法,则为15Hz
        data[7] = 0x00;
    
  }
  else if(type == 2)
  {
     if (spin_frequency_Hz == 12)
      data[7] = 0x00;
    else if (spin_frequency_Hz == 25)
      data[7] = 0x01;
    else if (spin_frequency_Hz == 50)
      data[7] = 0x02;
    else                                               // 若输入非法,则为12.5Hz
      data[7] = 0x00;


  }
  else if(type == 4)
  {
    if (spin_frequency_Hz == 10)
    data[7] = 0x00;
    else if (spin_frequency_Hz == 15)
        data[7] = 0x01;
    else if (spin_frequency_Hz == 20)
       data[7] = 0x02;
    else if (spin_frequency_Hz == 25)
        data[7] = 0x03;
    else if (spin_frequency_Hz == 30)
        data[7] = 0x04;
    else // 若输入非法,则为30kHz
    {
        data[7] = 0x04;
       
    }


  }
  else
  {
      if (spin_frequency_Hz == 15)
        data[7] = 0x00;
      else if (spin_frequency_Hz == 30)
        data[7] = 0x01;
      else                                               // 若输入非法,则为15Hz
        data[7] = 0x00;
  }
 

  //----------设置角度分辨率----------//
  if(type == 1)
  {
      if(angle_increment=="0.025")
      {
        data[7] = 0x00;
        data[8] = 0x00;
      }
      else if(angle_increment=="0.050")
      {
          data[8] = 0x01;
      }
      else if(angle_increment=="0.100")
      {
          data[8] = 0x02;
      }

      else if(angle_increment=="0.250")
      {
          data[8] = 0x03;
      }
      else if(angle_increment=="0.500")
      {
          data[8] = 0x04;
      }
      else                        // 都不符合则调到0.025/10hz
      {
          data[7] = 0x00;
          data[8] = 0x00;
      }
  }
  else if(type == 2)
  {
      //----------设置角度分辨率----------//
    if(angle_increment=="0.025")
    {
      data[7] = 0x00;
      data[8] = 0x00;
    }
    else if(angle_increment=="0.050")
    {
        data[8] = 0x01;
    }
    else if(angle_increment=="0.100")
    {
        data[8] = 0x02;
    }
    else if(angle_increment=="0.200")
    {
        data[8] = 0x03;
    }
    else if(angle_increment=="0.250")
    {
        data[8] = 0x04;
    }
    else if(angle_increment=="0.500")
    {
        data[8] = 0x05;
    }
    else                        // 都不符合则调到0.025/10hz
    {
        data[7] = 0x00;
        data[8] = 0x00;
    }
  }

  else if(type == 4)
  {
    if (sample_size_int < 0 || sample_size_int > 50) // 输入非法
    {
       
        sample_size_int = 1;
        data[8]  = (unsigned char)sample_size_int;
    } 
    else
        data[8] = (unsigned char)sample_size_int;
  }

  else
  {
        data[7] = 0x00;
        data[8] = 0x00;
  }
  
  //----------设置过滤等级----------//
  if ( noise_filter_level < 0 || noise_filter_level > 3)   // 输入非法，则默认为1
  {
   // ROS_WARN_STREAM("noise_filter_level error!");
    noise_filter_level = 1;
     data[9] = (unsigned char)noise_filter_level;
  }
  else
     data[9]  = (unsigned char)noise_filter_level;

  //----------设置校验位----------//
  CRC_Verify_len12(data);



  
}

void LasrProtocol::SetAreaCommand(HSGetAreaDataPackage command,vector<unsigned char> &data)
{  
  /**********帧头**********/
  data[0] = 0x57;
  data[1] = 0x53;
  data[2] = 0x69;
  data[3] = 0x6d;
  data[4] = 0x75;

 
  /**********传感器工作模式**********/
  data[5] = command.mode;

  /**********传感器通道值**********/
    if(command.channel > 63 | command.channel < 0)
    command.channel = 0;
  data[6] = command.channel;

  /**********通道角度**********/
  if(command.angle > 180 | command.angle < -180)
    command.angle = 0;
  data[7] = (command.angle& 0xff00) >> 8;       // 高位在前
  data[8] = (command.angle& 0x00ff);                // 低位在后

  /**********通道速度**********/
    if(command.speed > 300 | command.speed < -300)
    command.speed = 0;
  data[9] = (command.speed& 0xff00) >> 8;       // 高位在前
  data[10] = (command.speed& 0x00ff);                // 低位在后

  /**********传感器通道组号**********/
  if(command.channel_group > 4 | command.channel_group < 0)
    command.channel_group = 0;
  data[11]  = (command.channel_group& 0xff00) >> 8;       // 高位在前
  data[12]  = (command.channel_group& 0x00ff);                // 低位在后

  /**********预留位**********/
  data[13]  = 0x00;
  data[14]  = 0x00;
  data[15]  = 0x00;

  short int check = CRCVerify(data,data.size());
  /**********校验位**********/
  data[16] = (check& 0x00ff);                // 低位在前
  data[17] = (check& 0xff00) >> 8;       // 高位在后
}


short int LasrProtocol::CRCVerify(vector<unsigned char> &data, int len)
{
  int i, j;
  unsigned int wCrc = 0xffff;
  unsigned int wPolynom = 0xA001;
  /*-------------------------------------*/
  for (i = 0; i < len - 2; i++) // 后两位为校验位
  {
    wCrc ^= data[i];
    for (j = 0; j < 8; j++)
    {
      if (wCrc & 0x0001)
      {
        wCrc = (wCrc >> 1) ^ wPolynom;
      }
      else
      {
        wCrc = wCrc >> 1;
      }
    }
  }
  return wCrc;
}


double LasrProtocol::normalizeAngle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}


