#include <iostream>
#include <chrono>
#include <thread>

#include "UST10LX.h"
#include "ObstacleFinder.h"


void show(const std::array<int16_t,UST10LX::dataSize>& array)
{
    for(int i = 0;i<array.size();i+=10)
    {
        if(array.at(i) == -1)
        {
            std::cout << "x";
        }
        else if(array.at(i) > 200)
        {
            std::cout << ".";
        }
        else if(array.at(i) > 100)
        {
            std::cout << "o";
        }
        else if(array.at(i) > 50)
        {
            std::cout << "@";
        }
        else
        {
            std::cout << "O";
        }
    }
    std::cout << std::endl;
}

void demoDistance()
{

    UST10LX LiDAR = UST10LX();
    LiDAR.connect("192.168.0.10");

    while(true)
    {
        LiDAR.scan();
        show(LiDAR.getScan());
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

}
