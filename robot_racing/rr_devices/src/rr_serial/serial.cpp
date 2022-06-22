#include "rr_serial/serial.h"

#include <sys/time.h>
#include <termios.h>
#include <fcntl.h> // File control definitions
#include <memory.h>
#include <unistd.h> // UNIX standard function definitions

namespace rr_devices
{
    ///
    //! @brief Get the Termios Char Size Flag from the human readable character size.
    //!
    //! @param[in] char_size character size in integer.
    //! @return int termios CSIZE value. one of the CS5, CS6, CS7, and CS8
    ///
    int GetTermiosCharSizeFlag(const int &char_size);

    ///
    //! @brief Get the Termios Baud Rate Flag from human readable baudrate value.
    //! The values which is not a baudrate candidate is floored to the right one.
    //!
    //! @param[in] baudrate baudrate in integer.
    //! @return int termios baudrate flag value.
    ///
    int GetTermiosBaudRateFlag(const int &baudrate);

    ///
    //! @brief Pointer implementation
    //!
    ///
    struct SerialDevice::Impl
    {
        ///
        //! @brief Serial device name. e.g. /dev/ttyS0
        //!
        ///
        std::string device_name;
        ///
        //! @brief file descriptor for the device.
        //!
        ///
        int fd;
        ///
        //! @brief termios configuration for the serial device.
        //!
        ///
        termios tty;
    };

    SerialDevice::SerialDevice() : impl_(new Impl)
    {
        // default fd is set -1
        impl_->fd = -1;
    }

    SerialDevice::~SerialDevice()
    {
        Close();
    }

    bool SerialDevice::Open(const std::string &device_name)
    {
        impl_->device_name = device_name;

        // setting flags
        int oflag = 0;
        oflag |= O_RDWR;   // read and write mode
        oflag |= O_NOCTTY; // This will not be a controlling terminal

        //open device
        impl_->fd = open(device_name.c_str(), O_RDWR | O_NOCTTY);
        if (impl_->fd < 0)
        {
            return false;
        }
        return true;
    }

    void SerialDevice::Close()
    {
        if (impl_->fd < 0)
        {
            return;
        }
        close(impl_->fd);
        impl_->fd = -1;
    }

    ssize_t SerialDevice::Read(void *dst, const size_t &size)
    {
        // invalid file descriptor
        if (impl_->fd < 0)
        {
            return -1;
        }
        return read(impl_->fd, dst, size);
    }
    ssize_t SerialDevice::Write(const void *src, const size_t &size)
    {
        // invalid file descriptor
        if (impl_->fd < 0)
        {
            return -1;
        }
        return write(impl_->fd, src, size);
    }

    bool SerialDevice::Configure(const int &baud, const int &csize)
    {
        // clear tty and load current tty setting
        memset(&impl_->tty, 0, sizeof(impl_->tty));
        int ret = tcgetattr(impl_->fd, &impl_->tty);
        if (ret < 0)
        {
            return false;
        }

        // setup input modes
        impl_->tty.c_iflag &= ~IXON;   //Disable XON/XOFF flow control on output.
        impl_->tty.c_iflag &= ~IXOFF;  //Disable XON/XOFF flow control on input.
        impl_->tty.c_iflag &= ~IXANY;  //Typing any character won't restart stopped output.
        impl_->tty.c_iflag &= ~BRKINT; // BREAK on input will not cause a SIGINT
        impl_->tty.c_iflag &= ~ICRNL;  // Don't translate carriage return to newline on input

        // setup output modes
        impl_->tty.c_oflag &= ~OPOST; // Disable implementation-defined output processing.

        // setup control modes
        cfsetspeed(&impl_->tty, GetTermiosBaudRateFlag(baud)); // set baudrate
        impl_->tty.c_cflag &= ~CSIZE;                          //  clear char size bits
        impl_->tty.c_cflag |= GetTermiosCharSizeFlag(csize);   //  set character size bits
        impl_->tty.c_cflag &= ~PARENB;                         //  Disable parity bit check
        impl_->tty.c_cflag &= ~CSTOPB;                         // 1 stop bit
        impl_->tty.c_cflag &= ~CRTSCTS;                        // Disable RTS/CTS (hardware) flow control.
        impl_->tty.c_cflag |= CREAD;                           // Enable receiver.
        impl_->tty.c_cflag |= CLOCAL;                          // Ignore Modem Control lines

        // setup local modes
        impl_->tty.c_lflag &= ~ECHO;   // Don't echo input.
        impl_->tty.c_lflag &= ~ICANON; // Non-Canonical mode
        impl_->tty.c_lflag &= ~IEXTEN; // Disable implementation-defined input processing
        impl_->tty.c_lflag &= ~ISIG;   // Don't generate the signals from recieved characters.

        // set read with timeout mode.
        // you don't know how much characters will arrive.
        //    when read returns same value as buffer size, you need to check if more data is arrived.
        // you know how much characters arrived.
        // you know there were no arrived data for VTIME*0.1 seconds when timeout.
        impl_->tty.c_cc[VMIN] = 0;  // imediatly return at reading 1 byte.
        impl_->tty.c_cc[VTIME] = 1; //0.1 sec timeout

        // apply immediately.
        ret = tcsetattr(impl_->fd, TCSANOW, &impl_->tty);
        if (ret < 0)
        {
            return false;
        }
        return true;
    }

    int GetTermiosCharSizeFlag(const int &char_size)
    {
        // this is default
        int ret = CS8;

        if (char_size == 8)
        {
            ret = CS8;
        }
        else if (char_size == 7)
        {
            ret = CS7;
        }
        else if (char_size == 6)
        {
            ret = CS6;
        }
        else if (char_size == 5)
        {
            ret = CS5;
        }
        return ret;
    }

    int GetTermiosBaudRateFlag(const int &baudrate)
    {
        int ret = B0;
        if (baudrate < 50)
        {
            ret = B0;
        }
        else if (baudrate < 75)
        {
            ret = B50;
        }
        else if (baudrate < 110)
        {
            ret = B75;
        }
        else if (baudrate < 134)
        {
            ret = B110;
        }
        else if (baudrate < 150)
        {
            ret = B134;
        }
        else if (baudrate < 200)
        {
            ret = B150;
        }
        else if (baudrate < 300)
        {
            ret = B200;
        }
        else if (baudrate < 600)
        {
            ret = B300;
        }
        else if (baudrate < 1200)
        {
            ret = B600;
        }
        else if (baudrate < 1800)
        {
            ret = B1200;
        }
        else if (baudrate < 2400)
        {
            ret = B1800;
        }
        else if (baudrate < 4800)
        {
            ret = B2400;
        }
        else if (baudrate < 9600)
        {
            ret = B4800;
        }
        else if (baudrate < 19200)
        {
            ret = B9600;
        }
        else if (baudrate < 38400)
        {
            ret = B19200;
        }
        else if (baudrate < 57600)
        {
            ret = B38400;
        }
        else if (baudrate < 115200)
        {
            ret = B57600;
        }
        else if (baudrate < 230400)
        {
            ret = B115200;
        }
        else if (baudrate < 460800)
        {
            ret = B230400;
        }
        else if (baudrate < 500000)
        {
            ret = B460800;
        }
        else if (baudrate < 576000)
        {
            ret = B500000;
        }
        else if (baudrate < 921600)
        {
            ret = B576000;
        }
        else if (baudrate < 1000000)
        {
            ret = B921600;
        }
        else if (baudrate < 1152000)
        {
            ret = B1000000;
        }
        else if (baudrate < 1500000)
        {
            ret = B1152000;
        }
        else if (baudrate < 2000000)
        {
            ret = B1500000;
        }
        else if (baudrate < 2500000)
        {
            ret = B2000000;
        }
        else if (baudrate < 3000000)
        {
            ret = B2500000;
        }
        else if (baudrate < 3500000)
        {
            ret = B3000000;
        }
        else if (baudrate < 4000000)
        {
            ret = B3500000;
        }
        else
        {
            ret = B4000000;
        }
        return ret;
    }
}