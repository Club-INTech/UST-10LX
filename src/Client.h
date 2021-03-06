//
// Created by Teo-CD on 21/02/19.
//

#ifndef LIDAR_UST_10LX_CLIENT_H
#define LIDAR_UST_10LX_CLIENT_H

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <string.h>
#include <vector>
#include <signal.h>

#include "DataPoint.h"


class Client {
public:
    Client();
    Client(const std::string&, uint16_t);
    ~Client();
    bool connect();

    bool send(const std::string&);
    bool receive(std::string&,bool = true);

    explicit operator bool();

    static void dataToString(std::string&,const std::vector<DataPoint>&);
    static std::string dataToString(const std::vector<DataPoint>&);
private:

    /// Separates coordinates of two points
    static constexpr char pointSeparator = ';';
    /// Separates two sets of coordinates
    static constexpr char coordinatesSeparator = ':';
    /// Communication header inserted before the message
    static const std::string messageHeader;
    /// Communication terminator appended after the message
    static const std::string messageTerminator;

    /// Size of reception buffer
    static constexpr uint16_t bufferSize = 100;

    int m_clientSocket;
    std::string m_serverAddress;
    uint16_t m_serverPort;

    void clientDisconnect();
};


#endif //LIDAR_UST_10LX_CLIENT_H
