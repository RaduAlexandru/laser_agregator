#pragma once

//c++
#include <iosfwd>
#include <sstream>

//My stuff
#include "tiny-process-library/process.hpp"


class RosBagPlayer{
public:
    RosBagPlayer();
    RosBagPlayer(std::string args);
    void play(std::string args);
    void pause();
    bool is_paused();
    void kill();

private:
    TinyProcessLib::Process m_rosbag;
    bool m_paused;

};