#ifndef HK_CAMERA_H
#define HK_CAMERA_H

#include <QObject>
#include <QMutex>
#include <QDateTime>
#include <atomic>
#include <string>
#include <vector>
#include <queue>
#include <functional>

// 海康 SDK 头文件
#include "HCNetSDK.h"

namespace hk_camera
{

/**
 * @brief 摄像头登录参数
 */
struct CameraLoginInfo
{
    std::string ip;
    uint16_t    port = 8000;
    std::string username;
    std::string password;
};

/**
 * @brief 摄像头基本信息
 */
struct CameraDeviceInfo
{
    std::string serial_no;
    int         channel_count  = 0;
    int         start_channel  = 1;
    std::string device_name;
};

/**
 * @brief 视频帧数据 (线程安全队列元素)
 */
struct VideoFrame
{
    std::vector<uint8_t> data;
    DWORD                 data_type = 0;   // NET_DVR_SYSHEAD / NET_DVR_STREAMDATA
    int64_t               timestamp = 0;   // 毫秒时间戳
    int                   channel   = 1;
};

/**
 * @brief 报警事件数据
 */
struct AlarmEvent
{
    LONG    command       = 0;
    DWORD   alarm_type    = 0;
    int     channels[16]  = {0};  // 16 通道报警状态
    int64_t timestamp     = 0;
};

/**
 * @brief SDK 功能能力信息
 */
struct SdkAbility
{
    int max_real_play_count  = 0;
    int max_alarm_count      = 0;
    int max_voice_count      = 0;
};

/**
 * @brief 海康摄像头 SDK 封装类
 *
 * 职责:
 *   - 封装 HCNetSDK 所有 API 调用
 *   - 管理 SDK 回调和设备连接生命周期
 *   - 将 SDK 回调数据通过 Qt signals 发给 UI 和 ROS2 层
 *   - 所有 SDK 调用线程安全
 *
 * 使用方式 (单例):
 *   auto* cam = HKCamera::instance();
 *   cam->init();
 *   cam->login(info);
 *   cam->startRealPlay(1);
 *   // ... 使用 signals 接收数据
 *   cam->stopRealPlay();
 *   cam->logout();
 *   cam->cleanup();
 */
class HKCamera : public QObject
{
    Q_OBJECT

public:
    // ---- 单例 ----
    static HKCamera* instance();
    ~HKCamera();

    // ---- 生命周期 ----
    bool init();
    bool cleanup();

    // ---- 登录/登出 ----
    int  login(const CameraLoginInfo& info);
    bool logout(int user_id);

    // ---- 实时预览/取流 ----
    int  startRealPlay(int user_id, int channel, bool blocked = false);
    bool stopRealPlay(int play_handle);

    // ---- 抓图 ----
    bool captureJPEG(int user_id, int channel, int quality,
                     const std::string& save_path);

    // ---- 回放 ----
    int  findFiles(int user_id, int channel,
                   const NET_DVR_TIME_SEARCH_COND& start,
                   const NET_DVR_TIME_SEARCH_COND& end);
    int  findNextFile(int find_handle, NET_DVR_FINDDATA_V50& file_data);
    bool findClose(int find_handle);
    int  playBackByTime(int user_id, int channel,
                        const NET_DVR_TIME& start, const NET_DVR_TIME& end);

    // ---- 报警 ----
    int  setupAlarmChan(int user_id);
    bool closeAlarmChan(int alarm_handle);
    int  startListenAlarm(const std::string& local_ip, uint16_t port);

    // ---- 语音 ----
    int  startVoiceCom(int user_id);
    bool stopVoiceCom(int voice_handle);

    // ---- 云台控制 ----
    bool ptzControl(int user_id, int channel, DWORD command, DWORD stop = 0);
    bool ptzPreset(int user_id, int channel, DWORD cmd, DWORD index = 0);

    // ---- 云台绝对位置控制 ----
    bool ptzGotoPreset(int user_id, int channel, int preset_no);
    bool ptzSetPreset(int user_id, int channel, int preset_no);
    bool ptzControlWithSpeed(int user_id, int channel, DWORD command, DWORD speed, DWORD stop = 0);

    // ---- 硬件巡航 (设备端执行, 程序可退出) ----
    bool ptzCruiseAddPoint(int user_id, int channel, int route, int point_index, int preset_no);
    bool ptzCruiseSetDwell(int user_id, int channel, int route, int point_index, int dwell_sec);
    bool ptzCruiseSetSpeed(int user_id, int channel, int route, int point_index, int speed);
    bool ptzCruiseClearPoint(int user_id, int channel, int route, int point_index);
    bool ptzCruiseDeleteRoute(int user_id, int channel, int route);
    bool ptzCruiseStart(int user_id, int channel, int route);
    bool ptzCruiseStop(int user_id, int channel, int route);
    bool ptzCruiseQuery(int user_id, int channel, int route,
                        std::vector<int>& presets, std::vector<int>& dwells, std::vector<int>& speeds);
    bool ptzCruiseSetRoute(int user_id, int channel, int route,
                           const std::vector<int>& presets, int dwell_sec, int speed);
    bool ptzClearAllPresets(int user_id, int channel);

    // ---- 聚焦控制 ----
    bool getFocusMode(int user_id, int channel, NET_DVR_FOCUSMODE_CFG& cfg);
    bool setFocusMode(int user_id, int channel, const NET_DVR_FOCUSMODE_CFG& cfg);
    bool setAutoFocusMode(int user_id, int channel, BYTE autoFocusMode = 0);
    bool setManualFocus(int user_id, int channel, DWORD focusPos);

    // ---- 配置 ----
    template<typename T>
    bool getDVRConfig(int user_id, DWORD command, int channel, T* params, DWORD* ret_len);
    template<typename T>
    bool setDVRConfig(int user_id, DWORD command, int channel, const T* params);

    // ---- 设备信息 ----
    SdkAbility getSDKAbility();
    std::string getSDKVersion();
    int         getLastError();

    // ---- 设备连接状态 ----
    bool isLoggedIn(int user_id) const;

signals:
    // ---- 视频流信号 (从 SDK 回调线程发出) ----
    void videoFrameReady(const hk_camera::VideoFrame& frame);

    // ---- 报警信号 ----
    void alarmReceived(const hk_camera::AlarmEvent& alarm);

    // ---- 语音数据信号 ----
    void voiceDataReady(const std::vector<uint8_t>& data);

    // ---- 状态信号 ----
    void deviceDisconnected(int user_id);
    void deviceReconnected(int user_id);
    void errorOccurred(int error_code, const QString& message);

private:
    explicit HKCamera(QObject* parent = nullptr);

    // 禁止拷贝
    HKCamera(const HKCamera&) = delete;
    HKCamera& operator=(const HKCamera&) = delete;

    // ---- SDK 回调转发 ----
    static void CALLBACK s_realDataCallback(LONG handle, DWORD data_type,
                                           BYTE* buffer, DWORD size, void* user);
    static void CALLBACK s_alarmCallback(LONG command, NET_DVR_ALARMER* alarmer,
                                        char* alarm_info, DWORD len, void* user);
    static void CALLBACK s_voiceDataCallback(LONG handle, char* data,
                                            DWORD size, BYTE flag, void* user);
    static void CALLBACK s_exceptionCallback(DWORD type, LONG user_id,
                                            LONG handle, void* user);

    // ---- 线程安全 ----
    mutable QMutex mutex_;
    std::atomic<bool> initialized_{false};
};

} // namespace hk_camera

Q_DECLARE_METATYPE(hk_camera::VideoFrame)
Q_DECLARE_METATYPE(hk_camera::AlarmEvent)

#endif // HK_CAMERA_H
