/*
 * Logger.cpp
 *
 *  Created on: Jan 12, 2017
 *      Author: wilczynski
 */

#include <ctime>

#include "monolidar_fusion/Logger.h"

namespace Mono_Lidar {

void Logger::Log(MsgType type, std::string msg) {
    if (!_isEnabled)
        return;

    switch (type) {
    case MsgType::MethodStart:

        if (!this->_logFlagMethodNamesStart)
            return;
        Log(getDepthString() + "Method Start: " + msg);
        _methodDepth++;

        if (this->_logFlagTimeInterval) {
            std::clock_t now = std::clock();
            _methodTimes.push_back(now);
        }

        break;
    case MsgType::MethodEnd:

        if (!this->_logFlagMethodNamesEnd)
            return;
        _methodDepth--;

        if (this->_logFlagTimeInterval) {
            double timeStart = _methodTimes.back();
            double timeEnd = std::clock();
            _methodTimes.pop_back();
            double deltaTime = (double)(timeEnd - timeStart) * 1000.0 / CLOCKS_PER_SEC;

            Log(getDepthString() + "Method End (" + std::to_string(deltaTime) + "): " + msg);
        } else
            Log(getDepthString() + "Method End: " + msg);

        break;
    case MsgType::Break:
        if (!this->_logFlagMethodNamesEnd)
            return;
        _methodDepth--;
        Log(getDepthString() + "Method Break: " + msg);

        break;
    case MsgType::TimeInterval:

        if (!this->_logFlagTimeInterval)
            return;
        Log(getDepthString() + "Time Interval: " + msg);

        break;
    case MsgType::CheckPoint:

        if (!this->_logFlagCheckPoint)
            return;
        Log(getDepthString() + "Check: " + msg);

        break;
    case MsgType::Warning:

        if (!this->_logFlagWarning)
            return;
        Log(getDepthString() + "Warning: " + msg);

        break;
    case MsgType::Error:

        if (!this->_logFlagError)
            return;
        Log(getDepthString() + "ERROR: " + msg);

        break;

    case MsgType::Msg1:

        if (!(this->_logMsgLevel < 1))
            return;
        Log(getDepthString() + "Msg(1): " + msg);

        break;
    case MsgType::Msg2:

        if (!(this->_logMsgLevel < 2))
            return;
        Log(getDepthString() + "Msg(2): " + msg);

        break;
    case MsgType::Msg3:

        if (!(this->_logMsgLevel < 3))
            return;
        Log(getDepthString() + "Msg(3): " + msg);

        break;
    case MsgType::Msg4:

        if (!(this->_logMsgLevel < 4))
            return;
        Log(getDepthString() + "Msg(4): " + msg);

        break;
    case MsgType::Msg5:

        if (!(this->_logMsgLevel < 5))
            return;
        Log(getDepthString() + "Msg(5): " + msg);

        break;
    default:
        Log("Logging Error: MsgType unknown.");
        break;
    }
}

void Logger::Log(std::string msg) {
    if (this->_outFlagConsole) {
        std::cout << msg << std::endl;
    }

    if (this->_outFlagFile) {
        // TODO
    }
}

std::string Logger::getDepthString() {
    std::string str = "";

    for (int i = 0; i < _methodDepth; i++)
        str = str + " ";

    return str;
}

void Logger::PrintEigenVector(const Eigen::Vector3d& vec, const std::string& name) {
    std::cout << name << ": "
              << "X: " << vec.x() << ", Y: " << vec.y() << ", Z: " << vec.z() << std::endl;
}
}
