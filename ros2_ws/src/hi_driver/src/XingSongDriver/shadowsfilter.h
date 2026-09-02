#ifndef SHADOWSFILTER_H
#define SHADOWSFILTER_H
#include <iostream>
#include <vector>
#include <string>
#include <string.h>
#include "protoc.h"
#include <cmath>
#include <set>
using namespace std;



class ShadowsFilter
{
public:
    ShadowsFilter();

    void Init(int scan_num,  ShadowsFilterParam &shadows_filter_param_);

    void Start(ScanData& scan_data, int scan_num,ShadowsFilterParam &shadows_filter_param_);
private:
    int now_scan_num = 0;
    int last_scan_num = 0;

    std::vector<float> shadows_filter_threshold_min_;
    std::vector<float> shadows_filter_threshold_max_;
    set<int> shadows_del_index;
};

#endif // LasrProtocol_H
