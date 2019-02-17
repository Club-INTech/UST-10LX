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


/**
 * Sets up a TCP server waiting for connection from robot's high level
 * @param highLevelAddress Address to which the high level will connect to
 * @param highLevelPort Port the high level will connect to
 * @return Socket ID of high level if connection was successful
 */
int highLevelConnect(const std::string& highLevelAddress, uint16_t highLevelPort)
{
    // Setup the server socket
    int serverSocket = socket(AF_INET,SOCK_STREAM,IPPROTO_IP);

    if(!serverSocket)
    {
        std::cerr << "Could not open server socket" << std::endl;
        std::cerr << strerror(errno) << std::endl;
        exit(EXIT_FAILURE);
    }


    // Configure the server socket
    sockaddr_in serverSocketDescriptor{};
    serverSocketDescriptor.sin_family = AF_INET;
    serverSocketDescriptor.sin_port = htons(highLevelPort);
    serverSocketDescriptor.sin_addr.s_addr = inet_addr("127.0.0.1");

    // Set socket options to force address and port reuse if possible
    // Avoids "Address already in use" errors when binding
    int trueOption = 1;
    if(setsockopt(serverSocket,SOL_SOCKET,SO_REUSEADDR|SO_REUSEPORT,&trueOption,sizeof(trueOption)) < 0)
    {
        std::cerr << "Could not set socket options" << std::endl;
        std::cerr << strerror(errno) << std::endl;
        exit(EXIT_FAILURE);
    }


    // Bind the socket to the configured address and port
    if(bind(serverSocket,reinterpret_cast<sockaddr*>(&serverSocketDescriptor),sizeof(serverSocketDescriptor)) < 0)
    {
        std::cerr << "Could not bind socket to address " << highLevelAddress << ":" << highLevelPort << std::endl;
        std::cerr << strerror(errno) << std::endl;
        exit(EXIT_FAILURE);
    }


    // Start listening for inbound connections
    uint16_t connectionBacklog = 1;
    if(listen(serverSocket,connectionBacklog) < 0)
    {
        std::cerr << "Could not start listening on socket" << std::endl;
        std::cerr << strerror(errno) << std::endl;
        exit(EXIT_FAILURE);
    }


    // If there is an inbound connection, accept the connection and retrieve the client socket
    std::cout << "Waiting for connection on address " << highLevelAddress << ":" << highLevelPort << std::endl;
    int highLevelSocket = accept(serverSocket, nullptr, nullptr);

    // If connection failed, try until a connection is established
    while(highLevelSocket < 0)
    {
        std::cerr << "Could not connect to client" << std::endl;
        std::cerr << strerror(errno) << std::endl;
        std::cerr << "Trying again..." << std::endl;
        errno = 0;
        highLevelSocket = accept(serverSocket, nullptr, nullptr);
    }

    std::cout << "Connection successful !" << std::endl;

    shutdown(serverSocket,SHUT_RDWR);
    close(serverSocket);

    return(highLevelSocket);
}

int main() {

    // Connect to high level
    int highLevelSocket = highLevelConnect("127.0.0.1",17865);


    // Create a UST10-LX object then try to connect to it
    UST10LX LiDAR = UST10LX();

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

    // Start scanning and sending data to high level
    std::string dataOutput;
    while(true)
    {
        LiDAR.scan();
        ObstacleFinder::toString(dataOutput,finder.findObstacles(LiDAR.getDataPoints()));
        write(highLevelSocket,dataOutput.c_str(),dataOutput.length());
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }


    return 0;
}