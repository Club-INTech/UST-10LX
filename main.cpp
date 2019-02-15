#include <iostream>
#include "UST10LX.h"


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


int main() {
    UST10LX LiDAR = UST10LX();
    LiDAR.connect("192.168.0.10");
    LiDAR.scan();

//    std::array<int16_t,UST10LX::dataSize>& scan = LiDAR.getScan();
//    for(int16_t distance: scan)
//    {
//        std::cout << distance << ", ";
//    }
//    std::cout << std::endl;

    while(true)
    {
        LiDAR.scan();
        show(LiDAR.getScan());
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }


    return 0;
}