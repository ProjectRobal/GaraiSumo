/*
    DMP doesn't work properly.
Even after calibration it returns strange data.
Maybe something wrong with FIFO reading?
Did you forget about some settings?

Problem might be in init function,
frimaware is flashed wrong?
writeMeomoryBlock function.

*/

#pragma once

#include "MPU6050.hpp"

#define MPU6050_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU6050_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_RA_Z_FINE_GAIN      0x05

#define MPU6050_RA_BANK_SEL         0x6D
#define MPU6050_RA_MEM_START_ADDR   0x6E
#define MPU6050_RA_MEM_R_W          0x6F
#define MPU6050_RA_DMP_CFG_1        0x70
#define MPU6050_RA_DMP_CFG_2        0x71

#define MPU6050_DMP_MEMORY_BANKS        8
#define MPU6050_DMP_MEMORY_BANK_SIZE    256
#define MPU6050_DMP_MEMORY_CHUNK_SIZE   16

#define MPU6050_DMP_FIFO_PACKET_SIZE 42

#define MPU6050_RA_MOT_THR          0x1F
#define MPU6050_RA_MOT_DUR          0x20
#define MPU6050_RA_ZRMOT_THR        0x21
#define MPU6050_RA_ZRMOT_DUR        0x22

class DMP6050 : public MPU6050
{
    private:

    uint8_t buffer[MPU6050_DMP_FIFO_PACKET_SIZE];

    bool setMemoryBank(uint8_t bank, bool prefetchEnabled=false, bool userBank=false);

    bool setMemoryStartAddress(uint8_t addr);

    uint8_t readMemoryByte();

    bool writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address);

    //bool writeProgMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address);

    bool setDMPConfig1(uint8_t config);

    bool setDMPConfig2(uint8_t config);

    bool setOTPBankValid(bool en);

    bool getOTPBankValid();

    bool resetDMP();

    bool DMPEnable();

    bool DMPDisable();

    public:

    DMP6050(i2c_port_t i2c_port = I2C_NUM_0,uint8_t i2c_address=MPU6050_ADDRESS);

    bool DMPinit();

    bool start();

    bool stop(); 

    bool setMotionDetectionThreshold(uint8_t threshold);

    bool setZeroMotionDetectionThreshold(uint8_t threshold);

    bool setMotionDetectionDuration(uint8_t duration);

    bool setZeroMotionDetectionDuration(uint8_t duration);

    uint16_t GetFIFOPacketSize() const
    {
        return MPU6050_DMP_FIFO_PACKET_SIZE;
    }

    bool FIFOPacketReady() const;

    uint16_t readFIFOPacket(uint8_t *buffer) const;

    // vec a 4 dimensional vector to store quadriatic values
    void readQuadriatic(uint8_t* buffer,Vec4Df& vec);

    // convert quadriatic to euler angels, q is input 4D quadriatic, vec 3D output
    void readAngels(const Vec4Df& q,Vec3Df& vec);   

    // get gravity vector from quadriatic q
    void getGravity(const Vec4Df& q,Vec3Df& vec);

    // q - quadratic, gravity - a gravity vector, vec - output vector
    void getYawPitchRoll(const Vec4Df& q,const Vec3Df& gravity,Vec3Df& vec);

    // buffer a packet readed from DMP, aa - output raw acceleration vector
    void getAcceleration(uint8_t* buffer,Vec3Di& aa);

    // aa - raw acceleration vector, gravity - a gravity vector, out - output vector
    void getLinearAccel(const Vec3Di& aa,const Vec3Df& gravity,Vec3Df& out);

    // aa - linear acceleration vector, q - quadratic , out - output vector
    void getLinearAccelInWorld(const Vec3Df& aa,const Vec4Df& q,Vec3Df& out);
};
