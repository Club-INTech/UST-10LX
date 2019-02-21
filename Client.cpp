//
// Created by trotfunky on 21/02/19.
//

#include "Client.h"

const std::string Client::headerString = {0x21,0x21};

Client::Client() : Client("127.0.0.1",17865)
{}

Client::Client(const std::string& serverIp, uint16_t serverPort)
{
    m_clientSocket = -1;
    m_serverAddress = serverIp;
    m_serverPort = serverPort;
}

Client::~Client()
{
    if(m_clientSocket>=0)
    {
        shutdown(m_clientSocket,SHUT_RDWR);
        close(m_clientSocket);
    }
}

/**
 * Sets up a TCP server waiting for connection from robot's high level
 * @param serverAddress Address to which the client will connect to
 * @param serverPort Port the client will connect to
 * @return Socket ID of high level if connection was successful
 */
bool Client::connect()
{
    // Setup the server socket
    int serverSocket = socket(AF_INET,SOCK_STREAM,IPPROTO_IP);

    if(!serverSocket)
    {
        std::cerr << "Could not open server socket" << std::endl;
        std::cerr << strerror(errno) << std::endl;
        return(false);
    }


    // Configure the server socket
    sockaddr_in serverSocketDescriptor{};
    serverSocketDescriptor.sin_family = AF_INET;
    serverSocketDescriptor.sin_port = htons(m_serverPort);
    serverSocketDescriptor.sin_addr.s_addr = inet_addr(m_serverAddress.c_str());

    // Set socket options to force address and port reuse if possible
    // Avoids "Address already in use" errors when binding
    int trueOption = 1;
    if(setsockopt(serverSocket,SOL_SOCKET,SO_REUSEADDR|SO_REUSEPORT,&trueOption,sizeof(trueOption)) < 0)
    {
        std::cerr << "Could not set socket options" << std::endl;
        std::cerr << strerror(errno) << std::endl;
        return(false);
    }


    // Bind the socket to the configured address and port
    if(bind(serverSocket,reinterpret_cast<sockaddr*>(&serverSocketDescriptor),sizeof(serverSocketDescriptor)) < 0)
    {
        std::cerr << "Could not bind socket to address " << m_serverAddress << ":" << m_serverPort << std::endl;
        std::cerr << strerror(errno) << std::endl;
        return(false);
    }


    // Start listening for inbound connections
    uint16_t connectionBacklog = 1;
    if(listen(serverSocket,connectionBacklog) < 0)
    {
        std::cerr << "Could not start listening on socket" << std::endl;
        std::cerr << strerror(errno) << std::endl;
        return(false);
    }


    // If there is an inbound connection, accept the connection and retrieve the client socket
    std::cout << "Waiting for connection on address " << m_serverAddress << ":" << m_serverPort << std::endl;
    int clientSocket = accept(serverSocket, nullptr, nullptr);

    // If connection failed, try until a connection is established
    while(clientSocket < 0)
    {
        std::cerr << "Could not connect to client" << std::endl;
        std::cerr << strerror(errno) << std::endl;
        std::cerr << "Trying again..." << std::endl;
        errno = 0;
        clientSocket = accept(serverSocket, nullptr, nullptr);
    }

    std::cout << "Connection successful !" << std::endl;

    shutdown(serverSocket,SHUT_RDWR);
    close(serverSocket);

    m_clientSocket = clientSocket;

    return(true);
}

bool Client::send(const std::string& message)
{
    if(m_clientSocket == -1)
    {
        return(false);
    }

    int16_t sentBytes = ::write(m_clientSocket,message.c_str(),message.size());

    // If the pipe is broken or the write failed, disconnect properly
    if(signal(SIGPIPE,SIG_IGN) == SIG_ERR || sentBytes < 0)
    {
        std::cerr << "Client disconnected !" << std::endl;
        std::cerr << strerror(errno) << std::endl;
        errno = 0;

        shutdown(m_clientSocket,SHUT_RDWR);
        close(m_clientSocket);
        m_clientSocket = -1;
        return(false);
    }

    return(true);
}

/**
 * String is formed by concatenation of all obstacles separated by a ObstacleFinder::pointSeparator.
 * Different sets of coordinates are separated by a ObstacleFinder::coordinatesSeparator
 * @brief Converts obstacle vector to a string which can be sent to the High Level
 * @param dataToConvert vector of DataPoints that will be converted to std::string
 */
void Client::dataToString(std::string& dataString,const std::vector<DataPoint>& dataToConvert)
{
    dataString.clear();
    dataString.append(headerString);
    for(const DataPoint& point: dataToConvert)
    {
        dataString += std::to_string(point.distance);
        dataString += coordinatesSeparator;
        dataString += std::to_string(point.angle);
        dataString += pointSeparator;
    }

    dataString.pop_back();      // Removes the last pointSeparator
    dataString.append("\n");
}

/**
 * @brief Overload returning a string
 * @sa ObstacleFinder::toString(std::string&,const std::vector<DataPoint>&)
 * @param dataToConvert
 * @return Data converted to a string which can be sent
 */
std::string Client::dataToString(const std::vector<DataPoint>& dataToConvert)
{
    std::string dataString;
    dataToString(dataString,dataToConvert);
    return dataString;
}

Client::operator bool()
{
    return(m_clientSocket>=0);
}