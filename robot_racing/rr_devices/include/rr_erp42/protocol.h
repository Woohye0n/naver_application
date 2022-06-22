#ifndef RR_DEVICES_RR_ERP42_PROTOCOL_H_
#define RR_DEVICES_RR_ERP42_PROTOCOL_H_
#include <inttypes.h>
#include <cassert>

namespace rr_devices
{
    enum PacketSize : uint32_t
    {
        kP2HPacketSize = 20,
        kH2PPacketSize = 14
    };
    enum Mode : uint8_t
    {
        kModeAuto = 0,
        kModeManual = 1
    };
    enum Gear : uint8_t
    {
        kGearForward = 0,
        kGearNeutral = 1,
        kGearBackward = 2
    };
    enum EStop : uint8_t
    {

        kEStopOff = 0,
        kEStopOn = 1
    };

    ///
    //! @brief PCU to High level Controller packet parser
    //!
    ///
    class P2H
    {
    public:
        ///
        //! @brief Construct a new P2H
        ///
        P2H();
        ///
        //! @brief Destroy the P2H
        ///
        ~P2H();
        ///
        //! @brief Set the reading data start pointer.
        //!
        //! @param[in] data parsing start pointer
        ///
        void SetData(uint8_t *data);
        ///
        //! @brief Check if the start characters and finish characters are in the proper place.
        //!
        //! @return true this frame starts with 'S','T','X' and ends with 0x0D, 0x0A
        //! @return false this is not valid packet
        ///
        bool IsSTXEXT();
        ///
        //! @brief Get the Driving mode. The mode is defined as enumerator
        //!
        //! @return uint8_t Driving mode. kModeAuto = 0, kModeManual = 1
        ///
        uint8_t GetAorM();
        ///
        //! @brief Get the Emergency Stop flag. 0 means off, 1 mans on
        //!
        //! @return uint8_t  kEStopOff = 0, kEStopOn = 1
        ///
        uint8_t GetEStop();
        ///
        //! @brief Get the Gear status
        //!
        //! @return uint8_t Forward = 0, Neutral=1, Backward=2
        ///
        uint8_t GetGear();
        ///
        //! @brief Get the Speed RPM. this is actual wheel rpm.
        //! manual said this value should be in the range of [-250, 250]
        //!
        //! @return int16_t wheel rpm
        ///
        int16_t GetSpeedRPM();
        ///
        //! @brief Get the Steer direction.
        //! Negative value meams left stear.
        //! value = deg * 71 with maximum 4% error
        //!
        //! @return int16_t steer direction.
        ///
        int16_t GetSteer();
        ///
        //! @brief Get the Brake scale
        //!
        //! @return uint8_t 1 : no braking, 100: full braking
        ///
        uint8_t GetBrake();
        //
        ///
        //! @brief Get the Encoder value. The encoder ppr is 48.
        //!
        //! @return int32_t encoder pulse count.
        ///
        int32_t GetEncoder();

        ///
        //! @brief Get the Battery voltage
        //!
        //! @return uint16_t actual voltage x 10
        ///
        uint16_t GetBattery();
        ///
        //! @brief Get the Alive. this is sequence number.
        //!
        //! @return uint8_t sequence number
        ///
        uint8_t GetAlive();

    private:
        uint8_t *data_;
    };
    ///
    //! @brief High level controler to PCU packet genenrator
    //!
    ///
    class H2P
    {
    public:
        H2P();
        ~H2P();
        ///
        //! @brief Set the reading data start pointer.
        //!
        //! @param[in] data parsing start pointer
        ///
        void SetData(uint8_t *data);
        ///
        //! @brief Set the Start and finish characters to the data.
        //!
        ///
        void SetSTXEXT();
        ///
        //! @brief Set the Driving Mode.
        //!
        //! @param[in] mode one of the kModeAuto or kModeManual
        ///
        void SetAorM(Mode mode);

        ///
        //! @brief Set the emergency stop flag
        //!
        //! @param[in] estop one of the kEStopOn or kEStopOff
        ///
        void SetEStop(EStop estop);

        ///
        //! @brief Set the Gear. Choose one of the kGearForward, kGearNeutral kGearBackward,
        //!
        //! @param[in] gear Target gear
        ///
        void SetGear(Gear gear);
        ///
        //! @brief Set the motor speed raw input. value in range [0,1000]
        //!
        //! @param[in] speed
        ///
        void SetSpeedMotorRaw(uint16_t speed);
        ///
        //! @brief Set the Steer Steer direction.
        //! Negative value means left stear.
        //! value = deg * 71 with maximum 4% error
        //!
        //! @param[in] steer target steer angle in degree x 71
        ///
        void SetSteer(int16_t steer);
        ///
        //! @brief Set the Brake scale
        //!
        //! @param[in] brake brake scale. 1: No brake. 100: Full brake
        ///
        void SetBrake(uint8_t brake);
        ///
        //! @brief Set the Alive. this is sequence number of the packet
        //!
        //! @param[in] alive sequence number of the packet
        ///
        void SetAlive(uint8_t alive);
        ///
        //! @brief increase the Alive number. if the stored Alive value is 255, next is 0
        //!
        ///
        void UpdateAlive();

    private:
        uint8_t *data_;
    };
}
#endif //RR_DEVICES_RR_ERP42_PROTOCOL_H_