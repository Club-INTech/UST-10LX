//
// Created by Teo-CD on 13/02/19.
//

#include "UST10LX.h"


UST10LX::UST10LX()
{
    m_socketID = -1;
}


UST10LX::~UST10LX()
{
    if(m_socketID != -1)
    {
        std::string stopMeasures = "QT\n";

        // Try to stop the LiDAR before closing the socket
        while (::write(m_socketID, stopMeasures.c_str(), stopMeasures.size()) != stopMeasures.size()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        ::close(m_socketID);
    }
}

void UST10LX::connect(const std::string& ip)
{
    // TCP socket setup
    m_socketID = socket(AF_INET,SOCK_STREAM,IPPROTO_IP);
    m_socketDescriptor.sin_family = AF_INET;
    m_socketDescriptor.sin_port = htons(m_port);

    // String to IP address
    m_socketDescriptor.sin_addr.s_addr = inet_addr(ip.c_str());

    // Connect TCP socket
    ::connect(m_socketID,reinterpret_cast<sockaddr*>(&m_socketDescriptor),sizeof(m_socketDescriptor));
}

bool UST10LX::scan()
{
    return false;
}

int16_t UST10LX::charDecode(uint16_t start, uint8_t charLength)
{
    return 0;
}

const std::array<uint16_t, UST10LX::m_dataSize>& UST10LX::getScan()
{
    return(m_lastScan);
}

uint16_t UST10LX::read(uint16_t bytesToRead)
{
    uint16_t bytesRead = 0;
    char tmpChar = '\0', oldTmpChar ='\0';
    m_recieveBuffer.clear();

    if(bytesToRead)
    {
        while(bytesRead<bytesToRead)
        {
            bytesRead += ::read(m_socketID,&tmpChar,1);
            m_recieveBuffer.push_back(tmpChar);
        }
    }
    else
    {
        // Read until message end : "\n\n"
        while(tmpChar != '\n' && oldTmpChar != '\n')
        {
            oldTmpChar = tmpChar;
            bytesRead += ::read(m_socketID,&tmpChar,1);
            m_recieveBuffer.push_back(tmpChar);
        }
    }

    return(bytesRead);
}

bool UST10LX::write(const std::string& message) {
    uint16_t sentBytes = ::write(m_socketID,message.c_str(),message.size());
    sentBytes += ::write(m_socketID,"\n",1);

    return(sentBytes == message.size() + 1);
}