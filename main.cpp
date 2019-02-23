#include <iostream>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <chrono>
#include <thread>

#include "UST10LX.h"
#include "ObstacleFinder.h"
#include "Client.h"


/// Define different ways of sending data to the high level
enum class communicationMode
{
    RAW,        /**< Send raw DataPoints , without processing   */
    OBSTACLES,  /**< Send obstacle positions                    */
};

int main() {

    // Connect to high level
    Client highLevel = Client("127.0.0.1",17865);
    highLevel.connect();

    // Create a UST10-LX object with a -135° offset then try to connect to it
    // -135° aligns the forward direction of the robot with 0°
    UST10LX LiDAR = UST10LX(-135);

    std::cout << "Trying to connect to UST10LX" << std::endl;
    LiDAR.connect("192.168.0.10");

    while(!LiDAR)
    {
        std::cout << "Retrying connection in 5 seconds" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(5));
        LiDAR.connect("192.168.0.10");
    }

    std::cout << "Connection successful !" << std::endl;
    std::cout << "Starting scanning and sending of data" << std::endl;


    ObstacleFinder finder = ObstacleFinder(UST10LX::dataError);

    std::string dataOutput;
    std::string messageInput;
    communicationMode mode = communicationMode::OBSTACLES;
    // Start scanning and sending data to high level
    while(true)
    {
        LiDAR.scan();

        switch(mode)
        {
            case communicationMode::RAW:
                Client::dataToString(dataOutput,LiDAR.getDataPoints());
                break;
            case communicationMode::OBSTACLES:
                Client::dataToString(dataOutput,finder.findObstacles(LiDAR.getDataPoints()));
        }

        highLevel.send(dataOutput);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Read eventual incoming messages and change mode accordingly
        if(highLevel.receive(messageInput,false))
        {
            if(messageInput == "R")
            {
                mode = communicationMode::RAW;
            }
            else if(messageInput == "O")
            {
                mode = communicationMode::OBSTACLES;
            }
            else
            {
                std::cout << "Unrecognized message" << std::endl;
                std::cout << messageInput << std::endl;
            }
        }

        // In the event of a disconnection, stop sending and try to reconnect
        if(!highLevel)
        {
            std::cout << "Waiting for a new client..." << std::endl;
            highLevel.connect();
        }
    }

    return 0;
}