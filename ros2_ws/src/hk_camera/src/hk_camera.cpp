#include "hk_camera/hk_camera.h"
#include <cstring>

namespace hk_camera
{

// ============================================================
// 内部辅助：回调上下文 (user data 传 this 指针)
// ============================================================

HKCamera* HKCamera::instance()
{
    static HKCamera s_instance;
    return &s_instance;
}

HKCamera::HKCamera(QObject* parent)
    : QObject(parent)
{
    // 注册跨线程信号参数类型 (必须在 connect 之前调用)
    qRegisterMetaType<hk_camera::VideoFrame>("hk_camera::VideoFrame");
    qRegisterMetaType<hk_camera::AlarmEvent>("hk_camera::AlarmEvent");
}

HKCamera::~HKCamera()
{
    cleanup();
}

// ============================================================
// 生命周期
// ============================================================

bool HKCamera::init()
{
    if (initialized_) return true;

    NET_DVR_Init();
    NET_DVR_SetLogToFile(3, (char*)"./hk_log");

    initialized_ = true;
    return true;
}

bool HKCamera::cleanup()
{
    if (!initialized_) return true;

    NET_DVR_Cleanup();
    initialized_ = false;
    return true;
}

// ============================================================
// 登录/登出
// ============================================================

int HKCamera::login(const CameraLoginInfo& info)
{
    if (!initialized_) init();

    NET_DVR_USER_LOGIN_INFO login_info = {0};
    NET_DVR_DEVICEINFO_V40   dev_info = {0};

    login_info.bUseAsynLogin = false;
    login_info.wPort         = info.port;
    strncpy(login_info.sDeviceAddress, info.ip.c_str(), NET_DVR_DEV_ADDRESS_MAX_LEN - 1);
    strncpy(login_info.sUserName, info.username.c_str(), NAME_LEN - 1);
    strncpy(login_info.sPassword, info.password.c_str(), NAME_LEN - 1);

    int user_id = NET_DVR_Login_V40(&login_info, &dev_info);

    if (user_id < 0)
    {
        int err = NET_DVR_GetLastError();
        emit errorOccurred(err, QString("Login failed: %1").arg(err));
        return -1;
    }

    return user_id;
}

bool HKCamera::logout(int user_id)
{
    if (user_id < 0) return false;
    return NET_DVR_Logout_V30(user_id);
}

bool HKCamera::isLoggedIn(int user_id) const
{
    return user_id >= 0;
}

// ============================================================
// 实时预览/取流
// ============================================================

int HKCamera::startRealPlay(int user_id, int channel, bool blocked)
{
    NET_DVR_PREVIEWINFO preview_info = {0};
    preview_info.lChannel        = channel;
    preview_info.dwStreamType    = 0;    // 主码流
    preview_info.dwLinkMode      = 4;    // RTP/RTSP
    preview_info.bBlocked        = blocked ? 1 : 0;
    preview_info.bPassbackRecord = 1;
    preview_info.byProtoType     = 1;    // RTSP 协议 → 标准 H.264 码流
    preview_info.dwDisplayBufNum = 1;

    printf("[HK] startRealPlay: user=%d, channel=%d, blocked=%d\n",
           user_id, channel, blocked ? 1 : 0);
    fflush(stdout);

    int handle = NET_DVR_RealPlay_V40(user_id, &preview_info,
        s_realDataCallback, this);

    printf("[HK] startRealPlay result: handle=%d, err=%d\n",
           handle, NET_DVR_GetLastError());
    fflush(stdout);

    if (handle < 0)
    {
        int err = NET_DVR_GetLastError();
        emit errorOccurred(err, QString("StartRealPlay failed: %1").arg(err));
    }

    return handle;
}

bool HKCamera::stopRealPlay(int play_handle)
{
    if (play_handle < 0) return false;
    return NET_DVR_StopRealPlay(play_handle);
}

// ============================================================
// 抓图
// ============================================================

bool HKCamera::captureJPEG(int user_id, int channel, int quality,
                            const std::string& save_path)
{
    NET_DVR_JPEGPARA jpeg_param = {0};
    jpeg_param.wPicQuality = static_cast<WORD>(quality);  // 0=best, 2=worst
    jpeg_param.wPicSize    = 0;   // 原始分辨率

    return NET_DVR_CaptureJPEGPicture(user_id, channel,
        &jpeg_param, (char*)save_path.c_str());
}

// ============================================================
// 回放 / 文件查找
// ============================================================

int HKCamera::findFiles(int user_id, int channel,
                         const NET_DVR_TIME_SEARCH_COND& start,
                         const NET_DVR_TIME_SEARCH_COND& end)
{
    NET_DVR_FILECOND_V50 cond = {0};
    cond.struStreamID.dwChannel = channel;
    cond.dwFileType             = 0xff;
    cond.struStartTime          = start;
    cond.struStopTime           = end;

    return NET_DVR_FindFile_V50(user_id, &cond);
}

int HKCamera::findNextFile(int find_handle, NET_DVR_FINDDATA_V50& file_data)
{
    return NET_DVR_FindNextFile_V50(find_handle, &file_data);
}

bool HKCamera::findClose(int find_handle)
{
    return NET_DVR_FindClose_V30(find_handle);
}

int HKCamera::playBackByTime(int user_id, int channel,
                              const NET_DVR_TIME& start, const NET_DVR_TIME& end)
{
    NET_DVR_VOD_PARA vod_param = {0};
    vod_param.struBeginTime     = start;
    vod_param.struEndTime       = end;
    vod_param.struIDInfo.dwChannel = channel;
    vod_param.hWnd              = 0;

    return NET_DVR_PlayBackByTime_V40(user_id, &vod_param);
}

// ============================================================
// 报警
// ============================================================

int HKCamera::setupAlarmChan(int user_id)
{
    NET_DVR_SETUPALARM_PARAM_V50 alarm_param = {0};
    alarm_param.dwSize              = sizeof(alarm_param);
    alarm_param.byRetAlarmTypeV40   = TRUE;
    alarm_param.byRetDevInfoVersion = TRUE;
    alarm_param.byAlarmInfoType     = 1;
    alarm_param.bySupport           = 4;  // 按位: bit2=1

    int handle = NET_DVR_SetupAlarmChan_V50(user_id, &alarm_param,
        nullptr, 0);

    if (handle < 0)
    {
        int err = NET_DVR_GetLastError();
        emit errorOccurred(err, QString("SetupAlarmChan failed: %1").arg(err));
        return -1;
    }

    // 注册报警回调
    NET_DVR_SetDVRMessageCallBack_V51(0,
        s_alarmCallback, this);

    return handle;
}

bool HKCamera::closeAlarmChan(int alarm_handle)
{
    if (alarm_handle < 0) return false;
    return NET_DVR_CloseAlarmChan_V30(alarm_handle);
}

int HKCamera::startListenAlarm(const std::string& local_ip, uint16_t port)
{
    return NET_DVR_StartListen_V30(
        const_cast<char*>(local_ip.c_str()), port,
        s_alarmCallback, this);
}

// ============================================================
// 语音
// ============================================================

int HKCamera::startVoiceCom(int user_id)
{
    int handle = NET_DVR_StartVoiceCom_MR_V30(user_id, 0,
        s_voiceDataCallback, this);

    if (handle < 0)
    {
        int err = NET_DVR_GetLastError();
        emit errorOccurred(err, QString("StartVoiceCom failed: %1").arg(err));
    }

    return handle;
}

bool HKCamera::stopVoiceCom(int voice_handle)
{
    if (voice_handle < 0) return false;
    return NET_DVR_StopVoiceCom(voice_handle);
}

// ============================================================
// 云台控制
// ============================================================

bool HKCamera::ptzControl(int user_id, int channel, DWORD command, DWORD stop)
{
    return NET_DVR_PTZControl_Other(user_id, channel, command, stop);
}

bool HKCamera::ptzPreset(int user_id, int channel, DWORD cmd, DWORD index)
{
    return NET_DVR_PTZPreset_Other(user_id, channel, cmd, index);
}

// ============================================================
// 云台绝对位置控制
// ============================================================

bool HKCamera::ptzGotoPreset(int user_id, int channel, int preset_no)
{
    return NET_DVR_PTZPreset_Other(user_id, channel, GOTO_PRESET, preset_no);
}

bool HKCamera::ptzSetPreset(int user_id, int channel, int preset_no)
{
    return NET_DVR_PTZPreset_Other(user_id, channel, SET_PRESET, preset_no);
}

bool HKCamera::ptzControlWithSpeed(int user_id, int channel,
                                    DWORD command, DWORD speed, DWORD stop)
{
    return NET_DVR_PTZControlWithSpeed_Other(user_id, channel, command, stop, speed);
}

// ============================================================
// 硬件巡航 (设备端执行)
// ============================================================

bool HKCamera::ptzCruiseAddPoint(int user_id, int channel,
                                  int route, int point_index, int preset_no)
{
    // FILL_PRE_SEQ(30): 将预置点加入巡航路线
    return NET_DVR_PTZCruise_Other(user_id, channel,
        FILL_PRE_SEQ, (BYTE)route, (BYTE)point_index, (WORD)preset_no);
}

bool HKCamera::ptzCruiseSetDwell(int user_id, int channel,
                                   int route, int point_index, int dwell_sec)
{
    // SET_SEQ_DWELL(31): 设置巡航点驻留时间
    return NET_DVR_PTZCruise_Other(user_id, channel,
        SET_SEQ_DWELL, (BYTE)route, (BYTE)point_index, (WORD)dwell_sec);
}

bool HKCamera::ptzCruiseSetSpeed(int user_id, int channel,
                                   int route, int point_index, int speed)
{
    // SET_SEQ_SPEED(32): 设置巡航点转速
    return NET_DVR_PTZCruise_Other(user_id, channel,
        SET_SEQ_SPEED, (BYTE)route, (BYTE)point_index, (WORD)speed);
}

bool HKCamera::ptzCruiseClearPoint(int user_id, int channel,
                                     int route, int point_index)
{
    // CLE_PRE_SEQ(33): 从巡航路线中删除某个预置点
    return NET_DVR_PTZCruise_Other(user_id, channel,
        CLE_PRE_SEQ, (BYTE)route, (BYTE)point_index, 0);
}

bool HKCamera::ptzCruiseDeleteRoute(int user_id, int channel, int route)
{
    // DEL_SEQ(43): 删除整条巡航路线
    return NET_DVR_PTZCruise_Other(user_id, channel,
        DEL_SEQ, (BYTE)route, 0, 0);
}

bool HKCamera::ptzCruiseStart(int user_id, int channel, int route)
{
    // RUN_SEQ(37): 启动巡航
    return NET_DVR_PTZCruise_Other(user_id, channel,
        RUN_SEQ, (BYTE)route, 0, 0);
}

bool HKCamera::ptzCruiseStop(int user_id, int channel, int route)
{
    // STOP_SEQ(38): 停止巡航
    return NET_DVR_PTZCruise_Other(user_id, channel,
        STOP_SEQ, (BYTE)route, 0, 0);
}

bool HKCamera::ptzCruiseQuery(int user_id, int channel, int route,
                               std::vector<int>& presets, std::vector<int>& dwells,
                               std::vector<int>& speeds)
{
    NET_DVR_CRUISE_RET cruise_ret = {0};
    if (!NET_DVR_GetPTZCruise(user_id, channel, route, &cruise_ret))
        return false;

    presets.clear();
    dwells.clear();
    speeds.clear();
    for (int i = 0; i < 32; i++)
    {
        // PresetNum == 0 表示该槽位为空
        if (cruise_ret.struCruisePoint[i].PresetNum == 0)
            break;
        presets.push_back(cruise_ret.struCruisePoint[i].PresetNum);
        dwells.push_back(cruise_ret.struCruisePoint[i].Dwell);
        speeds.push_back(cruise_ret.struCruisePoint[i].Speed);
    }
    return true;
}

bool HKCamera::ptzCruiseSetRoute(int user_id, int channel, int route,
                                  const std::vector<int>& presets, int dwell_sec, int speed)
{
    // 先删除旧路线 (忽略失败, 可能路线本来就不存在)
    NET_DVR_PTZCruise_Other(user_id, channel, DEL_SEQ, (BYTE)route, 0, 0);

    // 逐个添加预置点
    bool all_ok = true;
    for (int i = 0; i < static_cast<int>(presets.size()); i++)
    {
        if (!NET_DVR_PTZCruise_Other(user_id, channel,
                FILL_PRE_SEQ, (BYTE)route, (BYTE)i, (WORD)presets[i]))
        {
            printf("[Cruise] FILL_PRE_SEQ failed: route=%d, idx=%d, preset=%d, err=%d\n",
                   route, i, presets[i], NET_DVR_GetLastError());
            all_ok = false;
        }
    }

    // 用 NET_DVR_CRUISE_PARA 配置驻留时间和速度 (比 SET_SEQ_DWELL 更可靠)
    NET_DVR_CRUISE_PARA cruise_para = {0};
    cruise_para.dwSize = sizeof(NET_DVR_CRUISE_PARA);
    for (int i = 0; i < static_cast<int>(presets.size()) && i < CRUISE_MAX_PRESET_NUMS; i++)
    {
        cruise_para.byPresetNo[i]    = (BYTE)presets[i];
        cruise_para.byCruiseSpeed[i] = (BYTE)speed;
        cruise_para.wDwellTime[i]    = (WORD)dwell_sec;
    }
    cruise_para.byEnableThisCruise = 1;

    if (!NET_DVR_SetDVRConfig(user_id, NET_DVR_SET_CRUISE, channel,
                               &cruise_para, sizeof(NET_DVR_CRUISE_PARA)))
    {
        printf("[Cruise] NET_DVR_SET_CRUISE failed: route=%d, err=%d\n",
               route, NET_DVR_GetLastError());
        // 配置 API 失败时, 回退用 SET_SEQ_DWELL 逐个设置
        for (int i = 0; i < static_cast<int>(presets.size()); i++)
        {
            NET_DVR_PTZCruise_Other(user_id, channel,
                SET_SEQ_DWELL, (BYTE)route, (BYTE)i, (WORD)dwell_sec);
            NET_DVR_PTZCruise_Other(user_id, channel,
                SET_SEQ_SPEED, (BYTE)route, (BYTE)i, (WORD)speed);
        }
    }

    return all_ok;
}

bool HKCamera::ptzClearAllPresets(int user_id, int channel)
{
    // CLE_ALL_PRESET(53): 清除所有预置点
    return NET_DVR_PTZPreset_Other(user_id, channel, CLE_ALL_PRESET, 0);
}

// ============================================================
// 聚焦控制
// ============================================================

bool HKCamera::getFocusMode(int user_id, int channel, NET_DVR_FOCUSMODE_CFG& cfg)
{
    cfg.dwSize = sizeof(NET_DVR_FOCUSMODE_CFG);
    DWORD ret_len = 0;
    return NET_DVR_GetDVRConfig(user_id, NET_DVR_GET_FOCUSMODECFG, channel,
                                &cfg, sizeof(NET_DVR_FOCUSMODE_CFG), &ret_len);
}

bool HKCamera::setFocusMode(int user_id, int channel, const NET_DVR_FOCUSMODE_CFG& cfg)
{
    return NET_DVR_SetDVRConfig(user_id, NET_DVR_SET_FOCUSMODECFG, channel,
                                const_cast<NET_DVR_FOCUSMODE_CFG*>(&cfg),
                                sizeof(NET_DVR_FOCUSMODE_CFG));
}

bool HKCamera::setAutoFocusMode(int user_id, int channel, BYTE autoFocusMode)
{
    // 先读取当前配置
    NET_DVR_FOCUSMODE_CFG cfg = {0};
    cfg.dwSize = sizeof(NET_DVR_FOCUSMODE_CFG);
    DWORD ret_len = 0;
    if (!NET_DVR_GetDVRConfig(user_id, NET_DVR_GET_FOCUSMODECFG, channel,
                              &cfg, sizeof(NET_DVR_FOCUSMODE_CFG), &ret_len))
    {
        // 读取失败时, 构造默认配置
        memset(&cfg, 0, sizeof(cfg));
        cfg.dwSize = sizeof(NET_DVR_FOCUSMODE_CFG);
    }

    cfg.byFocusMode     = 0;  // 自动
    cfg.byAutoFocusMode = autoFocusMode;  // 0=关闭, 1=A, 2=B, 3=AB, 4=C
    return NET_DVR_SetDVRConfig(user_id, NET_DVR_SET_FOCUSMODECFG, channel,
                                &cfg, sizeof(NET_DVR_FOCUSMODE_CFG));
}

bool HKCamera::setManualFocus(int user_id, int channel, DWORD focusPos)
{
    // 先读取当前配置
    NET_DVR_FOCUSMODE_CFG cfg = {0};
    cfg.dwSize = sizeof(NET_DVR_FOCUSMODE_CFG);
    DWORD ret_len = 0;
    if (!NET_DVR_GetDVRConfig(user_id, NET_DVR_GET_FOCUSMODECFG, channel,
                              &cfg, sizeof(NET_DVR_FOCUSMODE_CFG), &ret_len))
    {
        memset(&cfg, 0, sizeof(cfg));
        cfg.dwSize = sizeof(NET_DVR_FOCUSMODE_CFG);
    }

    cfg.byFocusMode = 1;       // 手动
    cfg.dwFocusPos  = focusPos; // [0x1000, 0xC000]
    return NET_DVR_SetDVRConfig(user_id, NET_DVR_SET_FOCUSMODECFG, channel,
                                &cfg, sizeof(NET_DVR_FOCUSMODE_CFG));
}

// ============================================================
// 配置 GET/SET
// ============================================================

template<typename T>
bool HKCamera::getDVRConfig(int user_id, DWORD command, int channel,
                             T* params, DWORD* ret_len)
{
    return NET_DVR_GetDVRConfig(user_id, command, channel,
        params, sizeof(T), ret_len);
}

template<typename T>
bool HKCamera::setDVRConfig(int user_id, DWORD command, int channel,
                             const T* params)
{
    return NET_DVR_SetDVRConfig(user_id, command, channel,
        const_cast<T*>(params), sizeof(T));
}

// ---- 显式模板实例化 (按需扩展) ----
template bool HKCamera::getDVRConfig<NET_DVR_COMPRESSIONCFG_V30>(
    int, DWORD, int, NET_DVR_COMPRESSIONCFG_V30*, DWORD*);
template bool HKCamera::setDVRConfig<NET_DVR_COMPRESSIONCFG_V30>(
    int, DWORD, int, const NET_DVR_COMPRESSIONCFG_V30*);
template bool HKCamera::getDVRConfig<NET_DVR_FOCUSMODE_CFG>(
    int, DWORD, int, NET_DVR_FOCUSMODE_CFG*, DWORD*);
template bool HKCamera::setDVRConfig<NET_DVR_FOCUSMODE_CFG>(
    int, DWORD, int, const NET_DVR_FOCUSMODE_CFG*);

// ============================================================
// SDK 信息
// ============================================================

SdkAbility HKCamera::getSDKAbility()
{
    SdkAbility ability = {0};
    NET_DVR_SDKABL sdk_abl = {0};

    if (NET_DVR_GetSDKAbility(&sdk_abl))
    {
        ability.max_real_play_count = sdk_abl.dwMaxRealPlayNum;
    }

    return ability;
}

std::string HKCamera::getSDKVersion()
{
    unsigned int ver = NET_DVR_GetSDKBuildVersion();
    char buf[64];
    snprintf(buf, sizeof(buf), "V%d.%d.%d.%d",
        (0xff000000 & ver) >> 24,
        (0x00ff0000 & ver) >> 16,
        (0x0000ff00 & ver) >> 8,
        (0x000000ff & ver));
    return std::string(buf);
}

int HKCamera::getLastError()
{
    return NET_DVR_GetLastError();
}

// ============================================================
// SDK 回调 → Qt signals 转发
// ============================================================

void CALLBACK HKCamera::s_realDataCallback(LONG handle, DWORD data_type,
                                            BYTE* buffer, DWORD size, void* user)
{
    auto* self = static_cast<HKCamera*>(user);
    if (!self || size == 0) return;

    // 调试: 每收到一帧就打印
    static int frame_count = 0;
    frame_count++;
    if (frame_count % 30 == 1 || data_type == NET_DVR_SYSHEAD)
    {
        printf("[HK] realDataCallback: handle=%d, type=%s, size=%u, frame=%d\n",
               (int)handle,
               (data_type == NET_DVR_SYSHEAD) ? "HEAD" :
               (data_type == NET_DVR_STREAMDATA) ? "DATA" : "OTHER",
               (unsigned)size, frame_count);
        fflush(stdout);
    }

    VideoFrame frame;
    frame.data.assign(buffer, buffer + size);
    frame.data_type = data_type;
    frame.timestamp = QDateTime::currentMSecsSinceEpoch();

    emit self->videoFrameReady(frame);
}

void CALLBACK HKCamera::s_alarmCallback(LONG command, NET_DVR_ALARMER* alarmer,
                                         char* alarm_info, DWORD len, void* user)
{
    auto* self = static_cast<HKCamera*>(user);
    if (!self) return;

    AlarmEvent event;
    event.command   = command;
    event.timestamp  = QDateTime::currentMSecsSinceEpoch();

    if (command == COMM_ALARM_V30 && alarm_info && len > 0)
    {
        NET_DVR_ALARMINFO_V30 alarm;
        memcpy(&alarm, alarm_info, sizeof(NET_DVR_ALARMINFO_V30));
        event.alarm_type = alarm.dwAlarmType;
        memcpy(event.channels, alarm.byChannel, sizeof(event.channels));
    }

    emit self->alarmReceived(event);
}

void CALLBACK HKCamera::s_voiceDataCallback(LONG handle, char* data,
                                             DWORD size, BYTE flag, void* user)
{
    auto* self = static_cast<HKCamera*>(user);
    if (!self || size == 0) return;

    std::vector<uint8_t> voice_data(data, data + size);
    emit self->voiceDataReady(voice_data);
}

void CALLBACK HKCamera::s_exceptionCallback(DWORD type, LONG user_id,
                                             LONG handle, void* user)
{
    auto* self = static_cast<HKCamera*>(user);
    if (!self) return;

    switch (type)
    {
    case EXCEPTION_RECONNECT:
        emit self->deviceReconnected(user_id);
        break;
    default:
        emit self->deviceDisconnected(user_id);
        break;
    }
}

} // namespace hk_camera
