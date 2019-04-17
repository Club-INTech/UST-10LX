//
// Created by Teo-CD on 13/02/19.
//

#include "UST10LX.h"
#include "ObstacleFinder.h"

/**
 * Constructor with optional argument to set angle offset
 * @param angleOffset optional, in °. Sets the offset between measured angle and returned angle
 */
UST10LX::UST10LX(int16_t angleOffset) : m_angleOffset(angleOffset)
{
    m_socketID = -1;
    m_dataPointScan.reserve(dataSize);
    for(int i=0;i<dataSize;i++)
    {
	DataPoint point;
	point.angle = 0.0;
	point.distance = 0;
        m_dataPointScan.push_back(point);
    }
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

UST10LX::operator bool() {
    return(m_socketID!=-1);
}

void UST10LX::connect(const std::string& ip)
{
    // TCP socket setup
    m_socketID = socket(AF_INET,SOCK_STREAM,IPPROTO_IP);
    m_socketDescriptor.sin_family = AF_INET;
    m_socketDescriptor.sin_port = htons(port);

    // String to IP address
    m_socketDescriptor.sin_addr.s_addr = inet_addr(ip.c_str());

    // Connect TCP socket
    if(::connect(m_socketID,reinterpret_cast<sockaddr*>(&m_socketDescriptor),sizeof(m_socketDescriptor)))
    {
        m_socketID = -1;
        std::cerr << "Connection to UST10LX on address " << ip << " failed !" << std::endl;
        std::cerr << strerror(errno) << std::endl;
        return;
    }

    // Wait a bit then start measurements
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    write("BM");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    read();
}

/**
 * The LiDAR performs a 270° scan. The data is then received, stripped of line feeds, checksums and control data then
 * decoded. At the same time, the distance data is stored along with the corresponding angles in radians.
 * @brief Perform a distance scan and retrieve data
 * @return true if scan was successful
 */
bool UST10LX::scan()
{
    // Perform a 270° scan
    if(!write("GD0000108000"))
    {
        return(false);
    }

    if(!read())
    {
        return(false);
    }

    /***********************
     * Message processing  *
     ***********************/

    // Look for the end of the response
    size_t foundPosition = m_receiveBuffer.find("\n\n");
    if(foundPosition == std::string::npos)       // If not found, return false
    {
        return(false);
    }
    m_receiveBuffer.erase(foundPosition-1,3);       // If it is, remove it and the preceding checksum


    // Keep only the data part of the answer
    m_receiveBuffer.erase(0,23);
    if(m_receiveBuffer.empty())                     // If there was not enough data, the scan failed
    {                                               // So return false
        return(false);
    }

    foundPosition = m_receiveBuffer.find('\n');     // Look for line feeds
    while(foundPosition != std::string::npos)
    {
        m_receiveBuffer.erase(foundPosition-1,2);   // While there are...
        foundPosition = m_receiveBuffer.find('\n'); // ... remove them and the preceding checksums
    }

    /*****************
     * Data decoding *
     *****************/

    m_lastScan.fill(-1);
    auto insertDistancePosition = m_lastScan.begin();
    auto insertDataPointPosition = m_dataPointScan.begin();
    m_dataPointCount = 0;
    // Loop through 3-bytes block, each one being an encoded data point
    for(uint16_t i = 0;i<m_receiveBuffer.size();i+=3)
    {
        int16_t tmpValue = charDecode(i,3);

        // Check if the data point is valid
        if(tmpValue < minDistance || tmpValue > maxDistance)
        {
            tmpValue = dataError;
            continue; // shh, no errors, only dreams now
        }

        float angle = ((i/3.0f)*0.25f+m_angleOffset)*degreeToRadian;

        const float angleRange = (90+10)*2 / 180.0f * PI;
        if(angle < -angleRange / 2 || angle > angleRange / 2) {
            continue;
        }

        *insertDistancePosition = tmpValue;
        ++insertDistancePosition;

        insertDataPointPosition->distance = tmpValue;
        insertDataPointPosition->angle = angle;
        ++insertDataPointPosition;

        m_dataPointCount++;
    }

    m_dataPointScan.resize(m_dataPointCount);

    return(true);
}

int16_t UST10LX::charDecode(uint16_t start, uint8_t charLength)
{
    int16_t value = 0;
    auto position = m_receiveBuffer.begin() + start;
    for(int i = 0;i<charLength;i++)
    {
        value += (*position-0x30) << 6*(charLength-i-1);
        ++position;
    }
    return(value);
}

const std::array<int16_t, UST10LX::dataSize>& UST10LX::getScan()
{
    return(m_lastScan);
}

const std::vector<DataPoint>& UST10LX::getDataPoints()
{
    return(m_dataPointScan);
}

uint16_t UST10LX::read(uint16_t bytesToRead)
{
    if(m_socketID == -1)
    {
        return(0);
    }

    uint16_t bytesRead = 0;
    char tmpChar = '\0', oldTmpChar ='\0';
    m_receiveBuffer.clear();

    if(bytesToRead)
    {
        while(bytesRead<bytesToRead)
        {
            bytesRead += ::read(m_socketID,&tmpChar,1);
            m_receiveBuffer.push_back(tmpChar);
        }
    }
    else
    {
        // Read until message end : "\n\n"
        while(tmpChar != '\n' || oldTmpChar != '\n')
        {
            oldTmpChar = tmpChar;
            bytesRead += ::read(m_socketID,&tmpChar,1);
            m_receiveBuffer.push_back(tmpChar);
        }
    }

    return(bytesRead);
}

bool UST10LX::write(const std::string& message)
{
    if(m_socketID == -1)
    {
        return(false);
    }

    std::string finalMessage = message;
    finalMessage.append("\n");

    uint16_t sentBytes = ::write(m_socketID,finalMessage.c_str(),finalMessage.size());

    return(sentBytes == finalMessage.size());
}

const uint32_t UST10LX::getDataPointCount()
{
    return m_dataPointCount;
}
