
#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Eigen>

namespace Mono_Lidar {
class Logger {
public:
    enum MsgType {
        MethodStart,
        MethodEnd,
        TimeInterval,
        CheckPoint,
        Break,
        Error,
        Warning,
        Msg1,
        Msg2,
        Msg3,
        Msg4,
        Msg5
    };

    static Logger& Instance() {
        static Logger _instance;
        return _instance;
    }
    ~Logger() {
    }

    inline void setEnabled(bool value) {
        _isEnabled = value;
    }
    inline void setLogMsgLevel(int value) {
        _logMsgLevel = value;
    }

    inline void setLogFlagMethodsNameStart(bool value) {
        _logFlagMethodNamesStart = value;
    }
    inline void setLogFlagMehodNameEnd(bool value) {
        _logFlagMethodNamesEnd = value;
    }
    inline void setLogFileMethodTimeDuration(bool value) {
        _logFlagMethodTimeDuration = value;
    }
    inline void setLogFlagTimeInterval(bool value) {
        _logFlagTimeInterval = value;
    }
    inline void setLogFlagCheckPoint(bool value) {
        _logFlagCheckPoint = value;
    }
    inline void setLogFlagWarning(bool value) {
        _logFlagWarning = value;
    }
    inline void setLogFlagError(bool value) {
        _logFlagError = value;
    }

    inline void setOutFlagConsole(bool value) {
        _outFlagConsole = value;
    }
    inline void setOutFlagFile(bool value) {
        _outFlagFile = value;
    }
    inline void setFilePath(std::string value) {
        _filePath = value;
    }
    void PrintEigenVector(const Eigen::Vector3d& vec, const std::string& name);

    void Log(MsgType type, std::string msg);
    void Log(std::string msg);

private:
    Logger() {
    }
    Logger(const Logger&);
    Logger& operator=(const Logger&);

    std::string getDepthString();

    // StartTime for the methods as First in last out
    std::vector<double> _methodTimes;


    // Flags if to specify which debug Types are active
    bool _isEnabled = false;
    int _logMsgLevel = 0;
    bool _logFlagMethodNamesStart = false;
    bool _logFlagMethodNamesEnd = false;
    bool _logFlagMethodTimeDuration = false;
    bool _logFlagTimeInterval = false;
    bool _logFlagCheckPoint = false;
    bool _logFlagWarning = false;
    bool _logFlagError = false;

    // debug output
    bool _outFlagConsole = false;
    bool _outFlagFile = false;

    std::string _filePath = "";

    // Misc
    int _methodDepth = 0;
};
}
