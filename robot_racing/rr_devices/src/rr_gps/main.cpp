#include "rr_gps/parser.h"
#include "rr_serial/serial.h"
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

int main(int argc, char *argv[])
{
    // init ros and get node handle with private namespace
    ros::init(argc, argv, "rr_gps");
    ros::NodeHandle nh("~");

    // get parameters
    std::string dev_path; // device path, default: /dev/ttyACM0
    int baud;             // baudrate for the device, default: 9600
    int buf_size;         // read buffer size. default:512
    nh.param<std::string>("dev_path", dev_path, "/dev/ttyACM0");
    nh.param<int>("baud", baud, 9600);
    nh.param<int>("buf_size", buf_size, 512);

    // create publisher
    ros::Publisher nav_pub = nh.advertise<sensor_msgs::NavSatFix>("nav", 3);

    // prepare device
    rr_devices::SerialDevice dev;

    // prepare parsers
    rr_devices::NMEASentence sentence;
    rr_devices::RMCParser rmc_parser;

    // create read buffer.
    std::vector<char> read_buf;
    read_buf.resize(buf_size);

    // open device and configure.
    ROS_ASSERT(dev.Open(dev_path));
    ROS_ASSERT(dev.Configure(baud, 8));

    // ros main loop.
    while (ros::ok())
    {
        // blocking read, timeout 0.1 sec, No minimun reading characters.
        int n = dev.Read(read_buf.data(), read_buf.size());
        // n : -1 for error, 0 for eof, n>1 for read btyes.
        if (n > 0)
        {
            // for each bytes.
            for (int i = 0; i < n; i++)
            {
                // parse NMEA sentence
                sentence.Put(read_buf[i]);
                if (!sentence.Parse())
                {
                    continue;
                }
                // parse RMC message
                if (rmc_parser.Parse(sentence.GetTockens()))
                {
                    rr_devices::RMCMessage &msg = rmc_parser.GetMessage();
                    sensor_msgs::NavSatFix nav;
                    nav.header = std_msgs::Header();
                    nav.latitude = msg.lat;
                    nav.longitude = msg.lon;
                    switch (msg.gnss_mode)
                    {
                    case rr_devices::RMCMessage::kNone:
                    case rr_devices::RMCMessage::kDeadReckoning:
                        nav.status.status = nav.status.STATUS_NO_FIX;
                        break;
                    case rr_devices::RMCMessage::kAutonomous:
                        nav.status.status = nav.status.STATUS_FIX;
                        break;
                    case rr_devices::RMCMessage::kDifferential:
                    case rr_devices::RMCMessage::kFloatRTK:
                        nav.status.status = nav.status.STATUS_SBAS_FIX;
                        break;
                    case rr_devices::RMCMessage::kFixedRTK:
                        nav.status.status = nav.status.STATUS_GBAS_FIX;
                        break;
                    default:
                        nav.status.status = nav.status.STATUS_NO_FIX;
                        break;
                    }
                    nav_pub.publish(nav);
                }
            }
        }
    }
}