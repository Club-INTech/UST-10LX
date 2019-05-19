//
// Created by trotfunky on 21/02/19.
//

#include "Client.h"
#include "ObstacleFinder.h"

const std::string Client::messageHeader = {0x21,0x21};
const std::string Client::messageTerminator = "\n";

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
        errno = 0;      // Reset errno after error processing
        return(false);
    }


    // Configure the server socket
    sockaddr_in serverSocketDescriptor{};
    serverSocketDescriptor.sin_family = AF_INET;
    serverSocketDescriptor.sin_port = htons(m_serverPort);
    serverSocketDescriptor.sin_addr.s_addr = htonl(INADDR_ANY);

    // Set socket options to force address and port reuse if possible
    // Avoids "Address already in use" errors when binding
    int trueOption = 1;
    if(setsockopt(serverSocket,SOL_SOCKET,SO_REUSEADDR|SO_REUSEPORT,&trueOption,sizeof(trueOption)) < 0)
    {
        std::cerr << "Could not set socket options" << std::endl;
        std::cerr << strerror(errno) << std::endl;
        errno = 0;
        return(false);
    }


    // Bind the socket to the configured address and port
    if(bind(serverSocket,reinterpret_cast<sockaddr*>(&serverSocketDescriptor),sizeof(serverSocketDescriptor)) < 0)
    {
        std::cerr << "Could not bind socket to address " << m_serverAddress << ":" << m_serverPort << std::endl;
        std::cerr << strerror(errno) << std::endl;
        errno = 0;
        return(false);
    }


    // Start listening for inbound connections
    uint16_t connectionBacklog = 1;
    if(listen(serverSocket,connectionBacklog) < 0)
    {
        std::cerr << "Could not start listening on socket" << std::endl;
        std::cerr << strerror(errno) << std::endl;
        errno = 0;
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
        clientDisconnect();
        return(false);
    }

    return(true);
}


/**
 * Receives data from client returned in receivedMessage. Reception can be non-blocking and will return false
 * at once if no data is available on function call.<br>
 * Message header and terminator are checked and stripped if valid <br>
 * receivedMessage is cleared on function call and can be returned empty.
 * @param receivedMessage : String in which data is returned (If there is any)
 * @param isBlocking : Allow for non-blocking read
 * @return false if there was an error or no data to read if read was non-blocking
 */
bool Client::receive(std::string& receivedMessage, bool isBlocking)
{
    receivedMessage.clear();

    if(m_clientSocket < 0)
    {
        return(false);
    }

    int blockingFlag;
    isBlocking ? blockingFlag = 0 : blockingFlag = MSG_DONTWAIT;   // Set the flag according to parameter

    char receptionBuffer[bufferSize];


    // Try to receive data
    ssize_t receivedLength = ::recv(m_clientSocket,receptionBuffer,bufferSize,blockingFlag);

    // If reception failed ...
    if(receivedLength < 0)
    {
        // ... check if it was expected (Non blocking socket that would block) or if there was an unexpected error
        if(isBlocking || !(errno & (EWOULDBLOCK | EAGAIN )) )
        {
            // If it was unexpected, print an error message
            std::cerr << "Error while trying to read from address " << m_serverAddress << ":" << m_serverPort << std::endl;
            std::cerr << strerror(errno) << std::endl;
        }
        errno = 0;
        return(false);
    }
    else if(receivedLength == 0) // Or, if we received a zero-length message, consider that the client disconnected
    {
        errno = ECONNRESET;
        clientDisconnect();
        return(false);
    }

    // Copy received data to the output string
    receivedMessage.assign(receptionBuffer,(uint16_t)receivedLength);

    // Check if the message has a valid header and terminator
    if(receivedMessage.find(messageHeader) != 0 ||
       receivedMessage.rfind(messageTerminator) != receivedLength - messageTerminator.length())
    {
        std::cerr << "Invalid message received" << std::endl;
        return(false);
    }

    // Strip the message from header and terminator
    receivedMessage = receivedMessage.substr(messageHeader.length(),
            receivedLength-messageHeader.length()-messageTerminator.length());

    return(true);
}


void Client::clientDisconnect()
{
    std::cerr << "Client disconnected !" << std::endl;
    std::cerr << strerror(errno) << std::endl;
    errno = 0;

    shutdown(m_clientSocket,SHUT_RDWR);
    close(m_clientSocket);
    m_clientSocket = -1;
}



/**
 * String is formed by concatenation of all obstacles separated by a ObstacleFinder::pointSeparator.<br>
 * Different sets of coordinates are separated by a ObstacleFinder::coordinatesSeparator<br>
 * Message header and terminator are inserted at the beginning and at the end respectively
 * @brief Converts obstacle vector to a string which can be sent to the High Level
 * @param dataToConvert vector of DataPoints that will be converted to std::string
 */
void Client::dataToString(std::string& dataString,const std::vector<DataPoint>& dataToConvert)
{
    dataString.clear();
    dataString.append(messageHeader);
    for(const DataPoint& point: dataToConvert)
    {
        dataString += std::to_string(point.distance);
        dataString += coordinatesSeparator;
        dataString += std::to_string(point.angle);
        dataString += pointSeparator;
    }

    dataString.pop_back();      // Removes the last pointSeparator
    dataString.append(messageTerminator);
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