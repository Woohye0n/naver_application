#include "rr_gps/parser.h"

namespace rr_devices
{

    ///
    //! @brief Parse the protocol
    //!
    //! @param[in] token protocol type token. e.g. "GNGGA", "GNRMC", ...
    //! @param[in] protocol target protocol. e.g. "GGA", "RMC", ...
    //! @return true protocol match the target.
    //! @return false protocol differ from the target.
    ///
    bool ParseProtocol(const std::string &token, const std::string &protocol)
    {
        return token.substr(2) == protocol;
    }
    ///
    //! @brief Parse UTC from the hhmmss.ss token
    //!
    //! @param[in] token token containing UTC time. format is hhmmss.ss
    //! @param[out] hour UTC hour
    //! @param[out] minute UTC minutes
    //! @param[out] sec UTC seconds including subseconds
    ///
    void ParseUTC(const std::string &token, int &hour, int &minute, float &sec)
    {
        hour = std::atoi(token.substr(0, 2).c_str());
        minute = std::atoi(token.substr(2, 2).c_str());
        sec = std::atof(token.substr(4).c_str());
    }
    ///
    //! @brief Parse latitude from ddmm.mmmm token. d means degree, m means minute.
    //!
    //! @param[in] token latitude token in ddmm.mmmm format.
    //! @param[out] degree calculated latitude in degrees
    ///
    void ParseLatitude(const std::string &token, double &degree)
    {
        const float &deg_parsed = std::atoi(token.substr(0, 2).c_str());
        const float &min_parsed = std::atof(token.substr(2).c_str());
        degree = deg_parsed + min_parsed / 60.0;
    }
    ///
    //! @brief Parse longitude from dddmm.mmmm token. d means degree, m means minute.
    //! notice that d is repeated 3 times, not 2 times.
    //! @param[in] token longitude token in dddmm.mmmm format.
    //! @param[out] degree calculated longitude in degrees.
    ///
    void ParseLongitude(const std::string &token, double &degree)
    {
        const float &deg_parsed = std::atoi(token.substr(0, 3).c_str());
        const float &min_parsed = std::atof(token.substr(3).c_str());
        degree = deg_parsed + min_parsed / 60.0;
    }
    ///
    //! @brief convert "N" to 1.0, "S" to -1.0.
    //!
    //! @param[in] token "N" or "S"
    //! @return double "N" to 1.0, "S" to -1.0.
    ///
    double NorthToSign(const std::string &token)
    {
        return (token == "N") ? 1.0 : -1.0;
    }
    ///
    //! @brief Convert "E" to 1.0, "W" to -1.0
    //!
    //! @param[in] token "E" or "W"
    //! @return double "E" to 1.0, "W" to -1.0
    ///
    double EastToSign(const std::string &token)
    {
        return (token == "E") ? 1.0 : -1.0;
    }

    ///
    //! @brief Parse Knot token. speed unit is knot.
    //!
    //! @param[in] token speed token.
    //! @param[out] knot speed in knots
    ///
    void ParseSpeedKnot(const std::string &token, float &knot)
    {
        knot = std::atof(token.c_str());
    }
    ///
    //! @brief convert speed unit from knots to the km/h
    //!
    //! @param[in] knot speed in knots.
    //! @return float speed in km/h
    ///
    float KnotToKph(const float &knot)
    {
        return 1.852 * knot;
    }
    ///
    //! @brief Parse UTC Date token. ddmmyy format. y means year, m means month, d means date
    //!
    //! @param[in] token ddmmyy foramtted string
    //! @param[out] date UTC date
    //! @param[out] month UTC month
    //! @param[out] year UTC year, but only 2 digits.
    ///
    void ParseDate(const std::string &token, int &date, int &month, int &year)
    {
        date = std::atoi(token.substr(0, 2).c_str());
        month = std::atoi(token.substr(2, 2).c_str());
        year = std::atoi(token.substr(4).c_str());
    }

    struct NMEASentence::Impl
    {
        ///
        //! @brief buffer for parsing
        ///
        std::vector<char> buf_;
        ///
        //! @brief parsed tokens.
        ///
        std::vector<std::string> tokens_;
        ///
        //! @brief checksum validity. if calculated checksum does not match, this value is false.
        ///
        bool checksum_valid_;
    };

    NMEASentence::NMEASentence() : impl_(new Impl){};
    NMEASentence::~NMEASentence(){};

    void NMEASentence::Put(const char &c)
    {
        impl_->buf_.push_back(c);
    }

    bool NMEASentence::Parse()
    {
        // start character
        if (impl_->buf_.front() != '$')
        {
            impl_->buf_.clear();
        }

        // finish character
        if (impl_->buf_.back() != '\n')
        {
            return false;
        }

        // remove all tockens
        impl_->tokens_.clear();

        // for chksum
        char chk = 0;
        bool is_body = false;
        bool is_chksum = false;
        for (auto &c : impl_->buf_)
        {
            // '$' is start character. set body flag.
            // let's create first string for a token.
            if (c == '$')
            {
                is_body = true;
                impl_->tokens_.push_back(std::string());
                continue;
            }
            // comma is delimiter. create new string for new token.
            if (c == ',')
            {
                impl_->tokens_.push_back(std::string());
                continue;
            }
            // asterisk is body finish character. unset body flag.
            // make last token: checksum. set chksum flag.
            if (c == '*')
            {
                is_body = false;
                is_chksum = true;
                impl_->tokens_.push_back(std::string());
                continue;
            }
            // only body message will be tokens, and be calculated for the checksum.
            if (is_body)
            {
                impl_->tokens_.back().push_back(c);
                chk ^= c;
                continue;
            }
            // last two characters are hex representation of checksum.
            if (is_chksum)
            {
                impl_->tokens_.back().push_back(c);
                continue;
            }
        }

        impl_->checksum_valid_ = (std::strtol(impl_->tokens_.back().c_str(), NULL, 16) == chk);
        impl_->buf_.clear();
        return true;
    };
    std::vector<std::string> &NMEASentence::GetTockens()
    {
        return impl_->tokens_;
    }

    struct RMCParser::Impl
    {
        RMCMessage msg;
    };

    RMCParser::RMCParser() : impl_(new Impl){};
    RMCParser::~RMCParser() {}
    bool RMCParser::Parse(const std::vector<std::string> &tokens)
    {
        if (!ParseProtocol(tokens[0], "RMC"))
        {
            return false;
        }
        ParseUTC(tokens[1], impl_->msg.hour, impl_->msg.min, impl_->msg.sec);
        ParseStatus(tokens[2], impl_->msg.status);
        ParseLatitude(tokens[3], impl_->msg.lat);
        impl_->msg.lat *= NorthToSign(tokens[4]);
        ParseLongitude(tokens[5], impl_->msg.lon);
        impl_->msg.lon *= EastToSign(tokens[6]);
        ParseSpeedKnot(tokens[7], impl_->msg.speed_knot);
        impl_->msg.speed_kph = KnotToKph(impl_->msg.speed_knot);
        impl_->msg.course_over_ground = std::atof(tokens[8].c_str());
        ParseDate(tokens[9], impl_->msg.date, impl_->msg.month, impl_->msg.year);
        ParseMode(tokens[12], impl_->msg.gnss_mode);
        return true;
    }

    RMCMessage &RMCParser::GetMessage()
    {
        return impl_->msg;
    }

    void RMCParser::ParseStatus(const std::string &token, bool &valid)
    {
        valid = (token == "A");
    }
    void RMCParser::ParseMode(const std::string &token, RMCMessage::GNSSMode &mode)
    {
        if (token == "A")
        {
            mode = RMCMessage::kAutonomous;
        }
        else if (token == "D")
        {
            mode = RMCMessage::kDifferential;
        }
        else if (token == "R")
        {
            mode = RMCMessage::kFixedRTK;
        }
        else if (token == "F")
        {
            mode = RMCMessage::kFloatRTK;
        }
        else if (token == "E")
        {
            mode = RMCMessage::kDeadReckoning;
        }
        else if (token == "N")
        {
            mode = RMCMessage::kNone;
        }
        else
        {
            // this should not happen.
            // throw // for debug.
        }
    }

}
