/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * imu_stage.cpp – IMU recording / synchronisation for rpicam-apps
 * 2025-05-11  –  extended with user-configurable output options
 */

#include <libcamera/stream.h>
#include "core/rpicam_app.hpp"
#include "post_processing_stages/post_processing_stage.hpp"
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <filesystem> 
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <cmath>
#include <sstream>
#include <iostream>
#include <fstream>
#include "ICM42688_registers.hpp"

#define IMU_ADDRESS 0x68
#define NAME "imu_sync"

// ────────────────────────────────────────────────────────────────────────────────
// Defaults – can be overridden from JSON
static constexpr double DEF_ACC_FS_G   = 4.0;    // g-full-scale  (ACC_CONFIG0 = 4 g)
static constexpr double DEF_GYRO_FS_DPS = 1000.0; // deg/s full-scale (GYRO_CONFIG0 = 1000 dps)
static constexpr double ADC_RESOLUTION  = 65535.0; // 16-bit ADC

constexpr double DEG2RAD = M_PI / 180.0;

using Stream = libcamera::Stream;
using Clock  = std::chrono::steady_clock;
using namespace std::chrono_literals;


/* ----- ICM-42688 FS-code helpers ----------------------------------------- */
static uint8_t accel_fs_code(double fs_g)
{
    if (fs_g <= 2.0)   return 0x03;   // ±2 g
    if (fs_g <= 4.0)   return 0x02;   // ±4 g
    if (fs_g <= 8.0)   return 0x01;   // ±8 g
    if (fs_g <= 16.0)  return 0x00;   // ±16 g
    throw std::runtime_error("Unsupported acc_scale (choose 2/4/8/16 g)");
}

static uint8_t gyro_fs_code(double fs_dps)
{
    if (fs_dps <= 250)   return 0x03;   // ±250 dps
    if (fs_dps <= 500)   return 0x02;   // ±500 dps
    if (fs_dps <= 1000)  return 0x01;   // ±1000 dps
    if (fs_dps <= 2000)  return 0x00;   // ±2000 dps
    throw std::runtime_error("Unsupported gyro_scale (choose 250/500/1000/2000 dps)");
}


class IMURecordingStage : public PostProcessingStage
{
public:
    explicit IMURecordingStage(RPiCamApp *app) : PostProcessingStage(app) {}

    // PP-stage framework -------------------------------------------------------
    char const *Name() const override;
    void        Read(boost::property_tree::ptree const &params) override;
    void        Configure() override;
    void        Start() override;
    void        Teardown() override;
    bool        Process(CompletedRequestPtr &completed_request) override;

private:
    // ─── IMU helpers ─────────────────────────────────────────────────────────
    std::string i2c_from_libcamera();
    int  write_reg(uint8_t reg, uint8_t data);
    int  read_multi_reg(uint8_t reg, uint8_t *data, size_t length);
    void initialize_imu();
    static int16_t to_int16(uint8_t msb, uint8_t lsb);

    // Decode & logging helpers
    void openCsv();
    void dump_fifo_raw(uint32_t frame_no, int64_t cam_ts);
    bool decode_imu_interp(int64_t cam_ts, uint32_t frame_no,
                           uint8_t *last_data, uint8_t *current_data);
    uint8_t acc_fs_code_{accel_fs_code(DEF_ACC_FS_G)};
    uint8_t gyro_fs_code_{gyro_fs_code(DEF_GYRO_FS_DPS)};
    // ─── state ───────────────────────────────────────────────────────────────
    Stream *stream_ {nullptr};
    int      i2c_fd {-1};

    std::string      filename_      {"imu_frame_log.csv"};
    std::string      i2c_device_path_      {"/dev/i2c-6"};
    std::ofstream    csv_;
    bool             print_console_ {false};
    bool             write_raw_     {false};

    // dynamic scaling (converted to factor “raw → physical”)
    double acc_factor_  {(DEF_ACC_FS_G   *2.0) / ADC_RESOLUTION};
    double gyro_factor_ {(DEF_GYRO_FS_DPS*2.0) / ADC_RESOLUTION};
};



char const *IMURecordingStage::Name() const
{
    return NAME;
}

// ────────────────────────────────────────────────────────────────────────────────
// JSON → internal parameters
void IMURecordingStage::Read(boost::property_tree::ptree const &params)
{
    filename_      = params.get<std::string>("filename",      filename_);
    print_console_ = params.get<bool>("print_console",        print_console_);
    write_raw_     = params.get<bool>("write_raw",            write_raw_);
    i2c_device_path_ = params.get<std::string>("i2c_device_path",      i2c_device_path_);

    double acc_fs_g   = params.get<double>("acc_scale",  DEF_ACC_FS_G);
    double gyro_fs_dps= params.get<double>("gyro_scale", DEF_GYRO_FS_DPS);

    // Convert full-scale ranges → register codes
    uint8_t acc_fs_sel  = accel_fs_code(acc_fs_g);
    uint8_t gyro_fs_sel = gyro_fs_code(gyro_fs_dps);

    // Cache for initialise_imu()
    acc_fs_code_  = acc_fs_sel;
    gyro_fs_code_ = gyro_fs_sel;

    acc_factor_  = (acc_fs_g   * 2.0) / ADC_RESOLUTION;
    gyro_factor_ = (gyro_fs_dps* 2.0) / ADC_RESOLUTION;
}

// ────────────────────────────────────────────────────────────────────────────────
void IMURecordingStage::Start()
{

}

// ────────────────────────────────────────────────────────────────────────────────
void IMURecordingStage::Teardown()
{
    write_reg(ICM42688_PWR_MGMT0,0x00);        // stop
    csv_.flush();
}

// ────────────────────────────────────────────────────────────────────────────────
void IMURecordingStage::Configure()
{
    stream_ = app_->GetMainStream();
    
    if (i2c_device_path_.empty() )
        throw std::runtime_error("IMU: cannot open with empty path");

    i2c_fd = open(i2c_device_path_.c_str(), O_RDWR);
    if (i2c_fd < 0)
        throw std::runtime_error("IMU: cannot open " + i2c_device_path_);

    std::cout << "[IMU] using " << i2c_device_path_ << '\n';

    initialize_imu();
    openCsv();
}

// ────────────────────────────────────────────────────────────────────────────────
bool IMURecordingStage::Process(CompletedRequestPtr &completed_request)
{
    auto buffer = completed_request->buffers[stream_];

    int64_t timestamp_ns =
        completed_request->metadata.get(controls::SensorTimestamp).value_or(
            buffer->metadata().timestamp);

    uint32_t frame_no = completed_request->sequence;

    if (write_raw_)
    {
        dump_fifo_raw(frame_no, timestamp_ns);
    }
    else
    {
        uint8_t last_sample[16]{};
        uint8_t current_sample[16]{};

        while (true)
        {
            if (read_multi_reg(ICM42688_FIFO_DATA, current_sample, 16) < 0)
                break;

            if (current_sample[0] == 0x6C)      // FSYNC sample
            {
                decode_imu_interp(timestamp_ns, frame_no, last_sample, current_sample);
                break;
            }
            else if (current_sample[0] == 0x68) // normal sample
            {
                memcpy(last_sample, current_sample, 16);
            }
            else if (current_sample[0] == 0xFF) // end of FIFO
                break;
        }
    }

    return false; // keep running other stages
}

// ────────────────────────────────────────────────────────────────────────────────
/* -------------------------------------------------------------------------
 *  Dump *all* fifo samples (raw → physical units, GCSV order)
 * -------------------------------------------------------------------------- */
void IMURecordingStage::dump_fifo_raw(uint32_t frame_no, int64_t cam_ts)
{
    /* 1.  Read FIFO byte count (2 B: high, low) ---------------------------- */
    uint8_t cnt[2]{};
    if (read_multi_reg(ICM42688_FIFO_COUNTH, cnt, 2) < 0)
        return;

    uint16_t fifo_bytes = (cnt[0] << 8) | cnt[1];
    if (fifo_bytes == 0)
        return;                     // nothing to do

    /* 2.  Read the whole FIFO into a temporary buffer ---------------------- */
    std::vector<uint8_t> fifo(fifo_bytes + 1);   // +1 for dummy header byte
    if (read_multi_reg(ICM42688_FIFO_DATA, fifo.data(), fifo_bytes) < 0)
        return;

    /* 3.  Walk through every 16-byte sample -------------------------------- */
    size_t offset = 0;
    while (offset + 15 < fifo.size())
    {
        uint8_t const *s = fifo.data() + offset;
        offset += 16;

        uint8_t header = s[0];
        if (header != 0x68 && header != 0x6C)
            continue;               // skip anything unexpected / padding

        uint16_t imu_timestamp = (s[14] << 8) | s[15];
        uint8_t imu_header = s[0];
        int16_t acc_x  = to_int16(s[1],  s[2]);
        int16_t acc_y  = to_int16(s[3],  s[4]);
        int16_t acc_z  = to_int16(s[5],  s[6]);
        int16_t gyro_x = to_int16(s[7],  s[8]);
        int16_t gyro_y = to_int16(s[9],  s[10]);
        int16_t gyro_z = to_int16(s[11], s[12]);

        double ax = acc_x  * acc_factor_;
        double ay = acc_y  * acc_factor_;
        double az = acc_z  * acc_factor_;
        double gx = gyro_x * gyro_factor_;
        double gy = gyro_y * gyro_factor_;
        double gz = gyro_z * gyro_factor_;

        if (print_console_)
            std::cout << "[RAW] F#" << frame_no
                      << " imu_ts="   << imu_timestamp
                      << " acc=("  << ax << "," << ay << "," << az << ")"
                      << " gyro=(" << gx << "," << gy << "," << gz << ")\n";

        csv_ << "0x"
             << std::hex << std::uppercase          // hexadecimal, upper-case A–F
             << std::setw(2) << std::setfill('0')   // always 2 digits, pad with 0
             << static_cast<int>(imu_header)        // promote to int for << operator
             << std::dec                            // ← back to normal decimal
             << ',' << imu_timestamp << ','
             << ax << ',' << ay << ',' << az << ','
             << gx << ',' << gy << ',' << gz << '\n';
    }
}


/* -----------  Interpolated decode (1 sample per frame)  ----------- */
bool IMURecordingStage::decode_imu_interp(int64_t cam_ts, uint32_t frame_no,
                                          uint8_t *last_data, uint8_t *current_data)
{
    // raw → int16
    auto acc16  = [&](uint8_t *d,int o){return to_int16(d[o+1],d[o+2]);};
    auto gyro16 = [&](uint8_t *d,int o){return to_int16(d[o+7],d[o+8]);};

    double time_between_samples = 1.0 / 200;      // 200 Hz ODR
    double ODR_us  = time_between_samples * 1e6;  // 5000 µs
    uint16_t FSYNC_delta = (current_data[14] << 8) | current_data[15];
    double factor = (ODR_us - FSYNC_delta) / ODR_us;

    auto lerp = [&](int16_t a,int16_t b)
                { return a + factor * (b - a); };

    double ax = lerp(acc16(last_data,0),  acc16(current_data,0)) * acc_factor_;
    double ay = lerp(acc16(last_data,2),  acc16(current_data,2)) * acc_factor_;
    double az = lerp(acc16(last_data,4),  acc16(current_data,4)) * acc_factor_;
    double gx = lerp(gyro16(last_data,0), gyro16(current_data,0)) * gyro_factor_;
    double gy = lerp(gyro16(last_data,2), gyro16(current_data,2)) * gyro_factor_;
    double gz = lerp(gyro16(last_data,4), gyro16(current_data,4)) * gyro_factor_;

    if (print_console_)
        std::cout << "F#" << frame_no
                  << " cam_ts=" << cam_ts
                  << " acc=("  << ax << "," << ay << "," << az << ")"
                  << " gyro=(" << gx << "," << gy << "," << gz << ")\n";

    csv_ << cam_ts     << ','            // t  (µs)
         << gx*DEG2RAD << ','                    // rad/s
         << gy*DEG2RAD << ','
         << gz*DEG2RAD << ','
         << ax         << ','                   // g
         << ay         << ','
         << az         << '\n';
    return true;
}

// ────────────────────────────────────  IMU low-level I²C  ─────────
int IMURecordingStage::write_reg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = { reg, data };
    i2c_msg msg    = { IMU_ADDRESS, 0, 2, buf };
    i2c_rdwr_ioctl_data io { &msg, 1 };
    return ioctl(i2c_fd, I2C_RDWR, &io);
}

int IMURecordingStage::read_multi_reg(uint8_t reg, uint8_t *data, size_t len)
{
    i2c_msg msgs[2];
    msgs[0] = { IMU_ADDRESS, 0,      1, &reg };
    msgs[1] = { IMU_ADDRESS, I2C_M_RD | I2C_M_NOSTART, static_cast<__u16>(len+1), data };
    i2c_rdwr_ioctl_data io { msgs, 2 };
    return ioctl(i2c_fd, I2C_RDWR, &io);
}

void IMURecordingStage::initialize_imu()
{
    write_reg(ICM42688_REG_BANK_SEL,      0x00);        // bank 0
    write_reg(ICM42688_DEVICE_CONFIG,     0x01); usleep(100000);

    // 200Hz ODR (0x0F) | FS bits in [5:4]
    write_reg(ICM42688_GYRO_CONFIG0,
              static_cast<uint8_t>((gyro_fs_code_ << 4) | 0x0F));

    write_reg(ICM42688_ACCEL_CONFIG0,
              static_cast<uint8_t>((acc_fs_code_  << 4) | 0x0F));


    write_reg(ICM42688_FIFO_CONFIG1,      0x0F);        // enable gyro+acc+temp+FSYNC
    write_reg(ICM42688_FIFO_CONFIG,       1<<6);        // start FIFO
    write_reg(ICM42688_FSYNC_CONFIG,      0x01);        // FSYNC on

    write_reg(ICM42688_REG_BANK_SEL,      0x01);
    write_reg(ICM42688_INTF_CONFIG5,      0x02);
    write_reg(ICM42688_REG_BANK_SEL,      0x00);

    write_reg(ICM42688_TMST_CONFIG,       0x23);        // timestamp rollover 24 bit
    write_reg(ICM42688_PWR_MGMT0,         0x0F);        // run
}

// ────────────────────────────────────────────────────────────────────────────────
void IMURecordingStage::openCsv()
{
    csv_.open(filename_, std::ios::out | std::ios::trunc);
    if (!csv_)
        throw std::runtime_error("IMU: cannot open " + filename_);
    if (write_raw_){
        csv_ << "RAW IMU CSV, header = 0x68 for normal timestamp, 0x6C for VSYNC delta in timestamp\n";
        csv_ << "header,t_imu,ax,ay,az,gx,gy,gz\n";        // <-- header row
    }
    else{
        /* --- GCSV mandatory header --- */
        csv_ << "GYROFLOW IMU CSV v0.5\n";
        csv_ << "t,gx,gy,gz,ax,ay,az\n";        // <-- header row
    }

}

// ────────────────────────────────────────────────────────────────────────────────
int16_t IMURecordingStage::to_int16(uint8_t msb, uint8_t lsb)
{
    return static_cast<int16_t>((msb << 8) | lsb);
}

// ────────────────────────────────────────────────────────────────────────────────
static PostProcessingStage *Create(RPiCamApp *app)
{
    return new IMURecordingStage(app);
}

static RegisterStage reg(NAME, &Create);
