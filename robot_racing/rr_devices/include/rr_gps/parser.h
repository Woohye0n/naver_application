#ifndef RR_DEVICES_RR_GPS_PARSER_H_
#define RR_DEVICES_RR_GPS_PARSER_H_

#include <memory>
#include <string>
#include <vector>

namespace rr_devices
{

    ///
    //! @brief RMC message definition.
    //!
    ///
    struct RMCMessage
    {
        ///
        //! @brief GNSS mode definition
        ///
        enum GNSSMode
        {

            kAutonomous,
            kDifferential,
            kFixedRTK,
            kFloatRTK,
            kDeadReckoning,
            kNone
        };
        ///
        //! @brief UTC year
        ///
        int year;
        ///
        //! @brief UTC month
        ///
        int month;
        ///
        //! @brief UTC date
        ///
        int date;
        ///
        //! @brief UTC hour
        ///
        int hour;
        ///
        //! @brief UTC minutes
        ///
        int min;
        ///
        //! @brief UTC seconds
        ///
        float sec;
        ///
        //! @brief fix status
        ///
        bool status;
        ///
        //! @brief latitude, positive to the North, negative to the south
        ///
        double lat;
        ///
        //! @brief longitude, positive to the East negative to the West
        ///
        double lon;
        ///
        //! @brief speed in knots
        ///
        float speed_knot;
        ///
        //! @brief speed in km/h. this is little sugar for you. not included in RMC protocol.
        ///
        float speed_kph;
        ///
        //! @brief course over ground in degree.
        ///
        float course_over_ground;
        ///
        //! @brief gnss mode of this message.
        ///
        GNSSMode gnss_mode;
    };
    ///
    //! @brief Grab NMEA sentence from byte stream.
    ///
    class NMEASentence
    {
    public:
        ///
        //! @brief Construct a new NMEASentence.
        ///
        NMEASentence();
        ///
        //! @brief Destroy the NMEASentence
        ///
        ~NMEASentence();
        ///
        //! @brief Put one charactor to the buffer.
        //!
        //! @param[in] c new incomming data.
        ///
        void Put(const char &c);
        ///
        //! @brief Parse sentensce from the buffer. need to check after every Put
        //!
        //! @return true Parser found a NMEA sentence and tokenized succesfully.
        //! @return false Parser didn't found a NMEA sentence.
        ///
        bool Parse();
        ///
        //! @brief Get the Tockens parsed from buffer.
        //!
        //! @return std::vector<std::string>& a vector of token strings. the last token is always a checksum.
        ///
        std::vector<std::string> &GetTockens();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };

    ///
    //! @brief RMC sentence parser from tokens
    ///
    class RMCParser
    {
    public:
        ///
        //! @brief Construct a new RMCParser
        ///
        RMCParser();
        ///
        //! @brief Destroy the RMCParser
        ///
        ~RMCParser();
        ///
        //! @brief Parse tokens and make a RMC message
        //!
        //! @param[in] tokens token strings.
        //! @return true RMC message is found. you can get RMCMessage using GetMessage().
        //! @return false RMC message is not found.
        ///
        bool Parse(const std::vector<std::string> &tokens);
        ///
        //! @brief Get the parsed RMCMessage
        //!
        //! @return RMCMessage& parsed RMC message.
        ///
        RMCMessage &GetMessage();

    private:
        ///
        //! @brief Parse status from token. A means valid, V means invalid.
        //!
        //! @param[in] token string "A" or "V"
        //! @param[out] valid true for "A", false for others.
        ///
        void ParseStatus(const std::string &token, bool &valid);
        ///
        //! @brief Parse the mode of gnss. corresponding modes are ;
        //! "A":kAutonomous, "D":kDifferential, "R": kFixedRTK, "F":kFloatRTK, "E":kDeadReckoning, "N":kNone
        //!
        //! @param[in] token mode token string. one of "A","D","R","F","E","N"
        //! @param[out] mode correspoinding RMCMessage::GNSSMode
        ///
        void ParseMode(const std::string &token, RMCMessage::GNSSMode &mode);

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };

}

#endif //RR_DEVICES_RR_GPS_PARSER_H_