#ifndef MAIN_HPP
#define MAIN_HPP

#include <iostream>
#include <fstream>
#include <boost/asio.hpp>
#include "crc32_mpeg.hpp"
#include <chrono>
#include <thread>
#include "exoskeleton.hpp"
#include "communication_embedded_system.hpp"
#include "lsa.hpp"

union FloatUInt8
{
    float   f;
    uint8_t b8[4];
};


#endif
