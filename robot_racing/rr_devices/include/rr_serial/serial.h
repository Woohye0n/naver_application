#ifndef RR_DEVICES_RR_SERIAL_SERIAL_H_
#define RR_DEVICES_RR_SERIAL_SERIAL_H_

#include <string>
#include <memory>

namespace rr_devices
{

    ///
    //! @brief Serial device class for reading with timout in non-canonical mode.
    //!
    ///
    class SerialDevice
    {
    public:
        ///
        //! @brief Construct a new Serial Device:: Serial Device
        //!
        ///
        SerialDevice();
        ///
        //! @brief Destroy the Serial Device:: Serial Device.
        //! close the device at destruction.
        ///
        ~SerialDevice();
        ///
        //! @brief open an new device with a device name.
        //! if you are curious about flags, see https://man7.org/linux/man-pages/man2/open.2.html
        //!
        //! @param[in] device_name device name.
        //! @return true device open succeed.
        //! @return false device is not opened properly.
        ///
        bool Open(const std::string &device_name);
        ///
        //! @brief Close the device. already closed device is not closed twice.
        //!
        ///
        void Close();
        ///
        //! @brief Read bytes from opend device
        //!
        //! @param[out] dst buffer pointer. read bytes will be copied here.
        //! @param[in] size maximum buffer size.
        //! @return ssize_t read size from the device. -1 means failure.
        ///
        ssize_t Read(void *dst, const size_t &size);
        ///
        //! @brief Write bytes to the opend device.
        //!
        //! @param[in] src source data pointer.
        //! @param[in] size size of the source data.
        //! @return ssize_t written size of the data to the device. -1 means failure.
        ///
        ssize_t Write(const void *src, const size_t &size);
        ///
        //! @brief Configure termios struct and apply it to the opened file descriptor.
        //! if you are curious about flags, see https://man7.org/linux/man-pages/man3/termios.3.html.
        //!
        //! @param[in] baud Baudrate in integer. this is human readable, not a flag. e.g. 9600, 115200, ...
        //! @param[in] csize Charactor size in integer.this is human readable, not a flag. e.g. 8, 7, ...
        //! @return true Configuration succeed.
        //! @return false Configuration failed.
        ///
        bool Configure(const int &baud, const int &csize);

    private:
        ///
        //! @brief Pointer implementation for this class
        //!
        ///
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };

}
#endif //RR_DEVICES_RR_SERIAL_SERIAL_H_