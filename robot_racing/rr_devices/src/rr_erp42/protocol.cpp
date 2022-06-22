#include "rr_erp42/protocol.h"

namespace rr_devices
{
    // packet from ERP42 to HLV
    P2H::P2H() : data_(nullptr) {}
    P2H::~P2H() {}
    void P2H::SetData(uint8_t *data)
    {
        data_ = data;
    }

    bool P2H::IsSTXEXT()
    {
        assert(data_);
        return (data_[0] == 'S') && (data_[1] == 'T') && (data_[2] == 'X') &&
               (data_[18] == 0x0D) && (data_[19] == 0x0A);
    }

    uint8_t P2H::GetAorM()
    {
        assert(data_);
        return data_[3];
    }
    uint8_t P2H::GetEStop()
    {
        assert(data_);
        return data_[4];
    }
    uint8_t P2H::GetGear()
    {
        assert(data_);
        return data_[5];
    }

    int16_t P2H::GetSpeedRPM()
    {
        assert(data_);
        int16_t speed = 0;
        speed |= data_[7];
        speed = (speed << 8) | data_[6];
        return speed;
    }
    // negative is left, value = deg * 71 with maximum 4% error
    int16_t P2H::GetSteer()
    {
        assert(data_);
        int16_t steer = 0;
        steer |= data_[9];
        steer = (steer << 8) | data_[8];
        return steer;
    }
    // 1 : no braking, 100: full braking
    uint8_t P2H::GetBrake()
    {
        assert(data_);
        return data_[10];
    }
    // ppr = 48
    int32_t P2H::GetEncoder()
    {
        assert(data_);
        int32_t encoder = 0;
        encoder |= data_[14];
        encoder = (encoder << 8) | data_[13];
        encoder = (encoder << 8) | data_[12];
        encoder = (encoder << 8) | data_[11];
        return encoder;
    }
    // voltage * 10
    uint16_t P2H::GetBattery()
    {
        assert(data_);
        uint16_t bat = 0;
        bat |= data_[16];
        bat = (bat << 8) | data_[15];
        return bat;
    }
    uint8_t P2H::GetAlive()
    {
        assert(data_);
        return data_[17];
    }

    H2P::H2P() : data_(nullptr)
    {
    }
    H2P::~H2P() {}

    void H2P::SetData(uint8_t *data)
    {
        data_ = data;
    }

    void H2P::SetSTXEXT()
    {
        assert(data_);
        data_[0] = 'S';
        data_[1] = 'T';
        data_[2] = 'X';
        data_[12] = 0x0D;
        data_[13] = 0x0A;
    }

    void H2P::SetAorM(Mode mode)
    {
        assert(data_);
        data_[3] = mode;
    }

    void H2P::SetEStop(EStop estop)
    {
        assert(data_);
        data_[4] = estop;
    }

    void H2P::SetGear(Gear gear)
    {
        assert(data_);
        data_[5] = gear;
    }
    void H2P::SetSpeedMotorRaw(uint16_t speed)
    {
        assert(data_);
        data_[7] = speed & 0xff;
        data_[6] = (speed >> 8);
    }

    //
    void H2P::SetSteer(int16_t steer)
    {
        assert(data_);
        data_[9] = steer & 0xff;
        data_[8] = (steer >> 8);
    }
    void H2P::SetBrake(uint8_t brake)
    {
        assert(data_);
        data_[10] = brake;
    }
    void H2P::SetAlive(uint8_t alive)
    {
        assert(data_);
        data_[11] = alive;
    }
    void H2P::UpdateAlive()
    {
        assert(data_);
        data_[11] = (data_[11] + 1) % 256;
    }
}