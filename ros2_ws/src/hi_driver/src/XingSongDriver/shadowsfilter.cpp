#include "shadowsfilter.h"

ShadowsFilter::ShadowsFilter()
{

}

void ShadowsFilter::Init(int scan_num,  ShadowsFilterParam &shadows_filter_param_)
{
  shadows_filter_threshold_max_.clear();
  shadows_filter_threshold_min_.clear();

  float max, min, min_angle, max_angle, angle_increment;
  angle_increment = 2*M_PI/(float)scan_num;
  int window;

  switch (shadows_filter_param_.shadows_filter_level)
  {
    case 1:                                         // 快速，筛选角度大,搜索窗口小
      min_angle = M_PI*5.0f/180.0f;                 // 转换为弧度制
      max_angle = M_PI*175.0f/180.0f;               // 转换为弧度制
      window = 1;
      shadows_filter_param_.window = window;
      break;

    case 2:                                         // 较慢，筛选角度大，搜索窗口适中
      min_angle = M_PI*5.0f/180.0f;                 // 转换为弧度制
      max_angle = M_PI*175.0f/180.0f;               // 转换为弧度制
      window = 3;
      shadows_filter_param_.window = window;
      break;

    case 3:                                         // 较慢，筛选角度较小,搜索窗口大
      min_angle = M_PI*15.0f/180.0f;                // 转换为弧度制
      max_angle = M_PI*165.0f/180.0f;               // 转换为弧度制
      window = 5;
      shadows_filter_param_.window = window;
      break;
    
    default:                                                           // 按照shadows_filter_param_配置
      min_angle = M_PI*shadows_filter_param_.min_angle/180.0f;         // 转换为弧度制
      max_angle = M_PI*shadows_filter_param_.max_angle/180.0f;         // 转换为弧度制
      window = shadows_filter_param_.window;

      break;
  }
#ifdef DEBUG
  std::cout << "shadows_filter_level:" << shadows_filter_param_.shadows_filter_level
      << "  min_angle:" << shadows_filter_param_.min_angle
      << "  max_angle:" << shadows_filter_param_.max_angle
      << std::endl;
  std::cout << "max:";
#endif
  for(int i=0;i < window;i++)
  {
    max = sin(max_angle)/sin(M_PI-max_angle-angle_increment*(i+1));
    min = sin(min_angle)/sin(M_PI-min_angle-angle_increment*(i+1));

    shadows_filter_threshold_max_.push_back(max);
    shadows_filter_threshold_min_.push_back(min);
    #ifdef DEBUG
    std::cout << shadows_filter_threshold_max_.back() << "  ";
    #endif
  }
}

void ShadowsFilter::Start(ScanData& scan_data, int scan_num,ShadowsFilterParam &shadows_filter_param_)
{
    if(scan_num != now_scan_num)
    {
       Init(scan_num,shadows_filter_param_);
       now_scan_num = scan_num;
       return;
    }

  int max=0;

  int search_index, search_index_tmp, del_index, del_index_tmp;
  float a_b_rate;

  shadows_del_index.clear();
  // 每traverse_step根激光计算一次
  for (int i = 0; i < scan_num; i += shadows_filter_param_.traverse_step)
  {
    if(scan_data.distance_data[i] > kMaxDistance)                        // 如果search_index_tmp激光超出范围，则跳过
      continue;

    for(search_index = i+1;                                              // 搜索[i,i+window]内是否存在拖影现象
        search_index <= i + shadows_filter_param_.window;
        search_index++)
    {
      search_index_tmp = search_index;  

      // 环形索引优化                                                         
      if(search_index_tmp < 0)                                                                        
        search_index_tmp = 0;
      else
        if(search_index_tmp >= scan_num)
          search_index_tmp = scan_num - 1;

      if( (scan_data.distance_data[search_index_tmp] > kMaxDistance) ||                               // 如果search_index_tmp激光超出范围，则跳过
          (scan_data.distance_data[i] > kMaxDistance)                  )
        continue;

      a_b_rate = (float)scan_data.distance_data[i]/(float)scan_data.distance_data[search_index_tmp];  // i与search_index激光比较
      if( (a_b_rate < shadows_filter_threshold_min_[abs(search_index_tmp-i-1)]) ||                      // 如果存在拖影现象
          (a_b_rate > shadows_filter_threshold_max_[abs(search_index_tmp-i-1)])   )        
        if (scan_data.distance_data[i] < scan_data.distance_data[search_index_tmp])
        {
          #ifdef DEBUG
          del_num++;
          #endif
          shadows_del_index.insert(search_index_tmp);
          if(abs(search_index_tmp-i-1)>max)
            max=abs(search_index_tmp-i-1);
        }
        else
        {
          if(abs(search_index_tmp-i-1)>max)
            max=abs(search_index_tmp-i-1);
          #ifdef DEBUG
          del_num++;
          #endif
          shadows_del_index.insert(i);
        }
    }
  }
  int index=0;
  for (auto iter = shadows_del_index.begin(); iter != shadows_del_index.end(); ++iter) 
  {
    scan_data.distance_data[*iter] = kMaxDistance + 1000;
  }
  #ifdef DEBUG
  std::cout << "del_num:" << del_num << std::endl;
  #endif
}