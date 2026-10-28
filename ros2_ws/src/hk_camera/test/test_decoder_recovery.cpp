#include "hk_camera/hk_decoder.h"
#include <gtest/gtest.h>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

namespace
{
struct Faults
{
    std::atomic<int> opens{0}, emitted{0};
    int fail_opens = 0;
    bool disconnect = false, empty = false, throws = false, stalls = false;
};

class FakeSource : public hk_camera::VideoSource
{
public:
    explicit FakeSource(std::shared_ptr<Faults> faults) : faults_(std::move(faults)) {}
    bool open(const std::string&, const hk_camera::DecoderOptions&) override
    {
        connection_ = ++faults_->opens;
        return connection_ > faults_->fail_opens;
    }
    bool read(cv::Mat& frame) override
    {
        std::this_thread::sleep_for(5ms);
        if (connection_ == 1) {
            if (faults_->throws) CV_Error(cv::Error::StsError, "rtsp://secret_user:secret_password@device");
            if (faults_->empty) return true;
            if (faults_->disconnect && ++reads_ > 2) return false;
            if (faults_->stalls) std::this_thread::sleep_for(50ms);
        }
        frame = cv::Mat(2, 2, CV_8UC3, cv::Scalar(connection_, 50, 100));
        return true;
    }
private:
    std::shared_ptr<Faults> faults_;
    int connection_ = 0, reads_ = 0;
};

struct Rig
{
    std::shared_ptr<Faults> faults = std::make_shared<Faults>();
    hk_camera::HKDecoder decoder{[this] { return std::make_unique<FakeSource>(faults); }};
    hk_camera::CameraLoginInfo login{"test-device", 8000, "secret_user", "secret_password"};
    Rig()
    {
        QObject::connect(&decoder, &hk_camera::HKDecoder::frameDecoded,
            [this](const QImage&) { ++faults->emitted; });
    }
    bool start(hk_camera::DecoderOptions options = {100, 100, 10})
    {
        return decoder.start(login, 1, options);
    }
    // Stop before members used by the signal callback are destroyed.
    ~Rig() { decoder.stop(); }
};

template<class Predicate>
bool waitFor(Predicate predicate)
{
    const auto end = std::chrono::steady_clock::now()+2s;
    while (std::chrono::steady_clock::now() < end) {
        if (predicate()) return true;
        std::this_thread::sleep_for(2ms);
    }
    return predicate();
}
}

TEST(DecoderRecovery, RetriesInitialOpenFailureWithoutAnotherStartRequest)
{
    Rig rig;
    rig.faults->fail_opens = 2;
    ASSERT_TRUE(rig.start());
    ASSERT_TRUE(waitFor([&] { return rig.faults->emitted >= 2; }));
    const auto health = rig.decoder.health();
    EXPECT_TRUE(rig.decoder.isRunning());
    EXPECT_TRUE(health.connected);
    EXPECT_EQ(health.connection_attempts, 3u);
    EXPECT_TRUE(health.last_error.empty());
    EXPECT_GE(health.frame_age_ms, 0);
}

TEST(DecoderRecovery, ReopensAfterAnEstablishedStreamDisconnects)
{
    Rig rig;
    rig.faults->disconnect = true;
    ASSERT_TRUE(rig.start());
    ASSERT_TRUE(waitFor([&] { return rig.decoder.health().frames >= 4; }));
    EXPECT_EQ(rig.faults->opens, 2);
    EXPECT_TRUE(rig.decoder.health().connected);
}

TEST(DecoderRecovery, EmptyFramesDoNotPermanentlyStopVideo)
{
    Rig rig;
    rig.faults->empty = true;
    ASSERT_TRUE(rig.start());
    ASSERT_TRUE(waitFor([&] { return rig.faults->emitted >= 2; }));
    EXPECT_EQ(rig.faults->opens, 2);
}

TEST(DecoderRecovery, RecoversFromExceptionsWithoutLoggingCredentials)
{
    Rig rig;
    rig.faults->throws = true;
    testing::internal::CaptureStderr();
    const bool started = rig.start();
    const bool recovered = waitFor([&] { return rig.faults->emitted >= 2; });
    rig.decoder.stop();
    const std::string log = testing::internal::GetCapturedStderr();
    ASSERT_TRUE(started);
    ASSERT_TRUE(recovered);
    EXPECT_NE(log.find("decode_error"), std::string::npos);
    EXPECT_EQ(log.find("secret_user"), std::string::npos);
    EXPECT_EQ(log.find("secret_password"), std::string::npos);
}

TEST(DecoderRecovery, DropsStalledReadAndReopensInsteadOfEmittingOldFrames)
{
    Rig rig;
    rig.faults->stalls = true;
    std::atomic<int> stale_frames{0};
    QObject::connect(&rig.decoder, &hk_camera::HKDecoder::frameDecoded,
        [&](const QImage& frame) { if (frame.pixelColor(0, 0).blue() == 1) ++stale_frames; });
    ASSERT_TRUE(rig.start({100, 20, 10}));
    ASSERT_TRUE(waitFor([&] { return rig.faults->emitted >= 2; }));
    rig.decoder.stop();
    EXPECT_EQ(stale_frames, 0);
    EXPECT_EQ(rig.faults->opens, 2);
}

TEST(DecoderRecovery, StopInterruptsRetryWaitAndDoesNotRestart)
{
    Rig rig;
    rig.faults->fail_opens = 100;
    ASSERT_TRUE(rig.start({100, 100, 5000}));
    ASSERT_TRUE(waitFor([&] { return rig.decoder.health().last_error == "open_failed"; }));
    const auto started = std::chrono::steady_clock::now();
    rig.decoder.stop();
    EXPECT_LT(std::chrono::steady_clock::now()-started, 100ms);
    const auto attempts = rig.faults->opens.load();
    std::this_thread::sleep_for(50ms);
    EXPECT_EQ(rig.faults->opens, attempts);
    EXPECT_FALSE(rig.decoder.isRunning());
    EXPECT_FALSE(rig.decoder.health().requested);
}

TEST(DecoderRecovery, ExplicitStartAfterStopCreatesOneNewReader)
{
    Rig rig;
    ASSERT_TRUE(rig.start());
    ASSERT_TRUE(waitFor([&] { return rig.faults->emitted >= 2; }));
    ASSERT_TRUE(rig.start());
    EXPECT_EQ(rig.faults->opens, 1);
    rig.decoder.stop();
    const auto emitted = rig.faults->emitted.load();
    ASSERT_TRUE(rig.start());
    ASSERT_TRUE(waitFor([&] { return rig.faults->emitted > emitted; }));
    EXPECT_EQ(rig.faults->opens, 2);
}
