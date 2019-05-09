// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <k4a/k4a.h>
#include <k4ainternal/common.h>
#include <utcommon.h>

#include <gtest/gtest.h>

// #include <k4ainternal/capturesync.h>
// #include <k4ainternal/capture.h>
// #include <azure_c_shared_utility/lock.h>
#include <azure_c_shared_utility/threadapi.h>
// #include <azure_c_shared_utility/condition.h>
#include <azure_c_shared_utility/envvariable.h>
#include <azure_c_shared_utility/tickcounter.h>

class capture_perf : public ::testing::Test

{
public:
    virtual void SetUp()
    {
        ASSERT_EQ(K4A_RESULT_SUCCEEDED, k4a_device_open(K4A_DEVICE_DEFAULT, &m_device)) << "Couldn't open device\n";
        ASSERT_NE(m_device, nullptr);
    }

    virtual void TearDown()
    {
        if (m_device != nullptr)
        {
            k4a_device_close(m_device);
            m_device = nullptr;
        }
    }

    k4a_device_t m_device = nullptr;
};

int main(int argc, char **argv)
{
    return k4a_test_commmon_main(argc, argv);
}

TEST_F(capture_perf, captureperf)
{
    k4a_wait_result_t wresult;
    k4a_capture_t capture;
    k4a_imu_sample_t imu = { 0 };
    TICK_COUNTER_HANDLE tick;
    const char *disable_sync = environment_get_variable("K4A_DISABLE_SYNCHRONIZATION");
    if (disable_sync == NULL || disable_sync[0] == '\0' || disable_sync[0] == '0')
    {
        ASSERT_TRUE(0) << "Environment variable K4A_DISABLE_SYNCHRONIZATION not set";
    }
    ASSERT_NE(tick = tickcounter_create(), (TICK_COUNTER_HANDLE)NULL);

    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
    // config.color_format = K4A_IMAGE_FORMAT_COLOR_NV12;
    // config.color_format = K4A_IMAGE_FORMAT_COLOR_YUY2;
    config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
    // config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_5;
    config.synchronized_images_only = false;
    ASSERT_EQ(K4A_RESULT_SUCCEEDED, k4a_device_start_cameras(m_device, &config));

    // start streaming.
    ASSERT_EQ(K4A_RESULT_SUCCEEDED, k4a_device_start_imu(m_device));

    //
    // Wait allow streams to start and then purge the data collected
    //
    if (config.camera_fps == K4A_FRAMES_PER_SECOND_30)
    {
        ThreadAPI_Sleep(2000);
    }
    else if (config.camera_fps == K4A_FRAMES_PER_SECOND_15)
    {
        ThreadAPI_Sleep(3000);
    }
    else
    {
        ThreadAPI_Sleep(4000);
    }
    while (K4A_WAIT_RESULT_SUCCEEDED == k4a_device_get_capture(m_device, &capture, 0))
    {
        // Drain the queue
        k4a_capture_release(capture);
    };
    while (K4A_WAIT_RESULT_SUCCEEDED == k4a_device_get_imu_sample(m_device, &imu, 0))
    {
        // Drain the queue
    };

    int32_t timeout_ms = 20000;
    tickcounter_ms_t start_time_tick;
    tickcounter_ms_t time_tick;

    uint64_t color_max = 0;
    uint64_t color_min = (uint64_t)(-1);
    uint64_t color_ave = 0;
    uint64_t color_ave_count = 0;
    uint64_t ir_max = 0;
    uint64_t ir_min = (uint64_t)(-1);
    uint64_t ir_ave = 0;
    uint64_t ir_ave_count = 0;

    ASSERT_EQ(0, tickcounter_get_current_ms(tick, &start_time_tick));

    do
    {
        do
        {
            // Get a capture that is not synchronized with another image
            wresult = k4a_device_get_capture(m_device, &capture, timeout_ms);
        } while (wresult == K4A_WAIT_RESULT_TIMEOUT);

        do
        {
            // Get the most recent IMU sample for correlated time
            wresult = k4a_device_get_imu_sample(m_device, &imu, 0);
        } while (K4A_WAIT_RESULT_TIMEOUT != wresult);

        k4a_image_t color = k4a_capture_get_color_image(capture);
        k4a_image_t ir = k4a_capture_get_ir_image(capture);

        // 1 image should be NULL because of K4A_DISABLE_SYNCHRONIZATION=1
        ASSERT_TRUE(ir == NULL || color == NULL);

        if (color)
        {
            uint64_t ts = k4a_image_get_timestamp_usec(color);
            uint64_t delta_ms = (imu.acc_timestamp_usec / 1000) - (ts / 1000);
            printf("Color:%6lld ms, IMU:%6lld ms delta:%3lld ms "
                   "IR:       ms, IMU:       ms delta:    ms \n",
                   ts / 1000,
                   imu.acc_timestamp_usec / 1000,
                   delta_ms);
            color_max = std::max(color_max, delta_ms);
            color_min = std::min(color_min, delta_ms);
            color_ave += delta_ms;
            color_ave_count++;
        }
        else
        {
            uint64_t ts = k4a_image_get_timestamp_usec(ir);
            uint64_t delta_ms = (imu.acc_timestamp_usec / 1000) - (ts / 1000);
            printf("Color:       ms, IMU        ms delta:    ms "
                   "IR:%6lld ms, IMU:%6lld ms delta:%3lld ms\n",
                   ts / 1000,
                   imu.acc_timestamp_usec / 1000,
                   delta_ms);
            ir_max = std::max(ir_max, delta_ms);
            ir_min = std::min(ir_min, delta_ms);
            ir_ave += delta_ms;
            ir_ave_count++;
        }

        if (color)
        {
            k4a_image_release(color);
        }
        if (ir)
        {
            k4a_image_release(ir);
        }
        if (capture)
        {
            k4a_capture_release(capture);
        }

        ASSERT_EQ(0, tickcounter_get_current_ms(tick, &time_tick));
    } while ((time_tick - start_time_tick) < 5000);

    /* IMU is the highest frequency data the device produces so we use that as an approimate measurement of time. We
    compare that time to what is used with depth and color to report latency. For color, depth and IR images the device
    timestamp is the center of exposure. so ta to see Accuracy is based on IMU rate and the number of samples batched
    together. so if the rate is 1666Hz and we get 8 IMU samples per batch then the variability in IMU timestamps should
    be about 5ms.
    */
    printf(
        "\nColor latency MIN:%lld ms MAX:%lld ms AVE:%lld ms       IR latency MIN:%lld ms MAX:%lld ms AVE:%lld ms\n\n",
        color_min,
        color_max,
        color_ave / color_ave_count,
        ir_min,
        ir_max,
        ir_ave / ir_ave_count);

    tickcounter_destroy(tick);
}
