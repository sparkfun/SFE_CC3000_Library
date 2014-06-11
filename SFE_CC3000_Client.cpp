/**
 * @file    SFE_CC3000_Client.cpp
 * @brief   Library for the SparkFun CC3000 shield and breakout boards
 * @author  Shawn Hymel (SparkFun Electronics)
 * 
 * @copyright   This code is public domain but you buy me a beer if you use
 * this and we meet someday (Beerware license).
 * 
 * The client library provides functions to connect to servers using sockets.
 */
 
#include <Arduino.h>
#include <SPI.h>
 
#include "common.h"
#include "SFE_CC3000.h"
#include "SFE_CC3000_Client.h"
#include "utility/socket.h"

 /**
  * @brief Constructor - Instantiates SFE_CC3000_Client object
  */
SFE_CC3000_Client::SFE_CC3000_Client(SFE_CC3000 &cc3000)
{
    cc3000_ = &cc3000;
    socket_ = -1;
}

/**
 * @brief Destructor
 */
SFE_CC3000_Client::~SFE_CC3000_Client()
{

}

/**
 * @brief Connects to a remote server using TCP
 *
 * @param[in] hostname the address of the remote server
 * @param[in] port the receiving port of the server (default: 80)
 * @return True if connected to remote server. False otherwise.
 */
int SFE_CC3000_Client::connect(    const char *hostname, 
                                    uint16_t port)
{
    IPAddress remote_ip;

    /* If CC3000 is not connected to a network, return false. */
    if (    !cc3000_->getInitStatus() || 
            !cc3000_->getConnectionStatus() || 
            !cc3000_->getDHCPStatus() ) {
        return false;
    }
    
    /* Perform a DNS lookup of the site */
    if (!cc3000_->dnsLookup(const_cast<char *>(hostname), &remote_ip)) {
        return false;
    }
    
    /* Connect to remote host using IP address */
    return connect(remote_ip, port);
}

/**
 * @brief Connects to a remote server using TCP
 *
 * @param[in] IP address of the remote server
 * @param[in] port the receiving port of the server (default: 80)
 * @return True if connected to remote server. False otherwise.
 */
int SFE_CC3000_Client::connect(    IPAddress ip_address, 
                                    uint16_t port)
{
    sockaddr dest_addr;
    int i;
                                    
    /* If CC3000 is not connected to a network, return false. */
    if (    !cc3000_->getInitStatus() || 
            !cc3000_->getConnectionStatus() || 
            !cc3000_->getDHCPStatus() ) {
        return false;
    }
    
    /* Create a socket */
    socket_ = socket(AF_INET, SOCK_STREAM, TCP);
    if (socket_ == -1) {
        return false;
    }
    
    /* Set address family to AF_INET (only one that works right now) */
    dest_addr.sa_family = AF_INET;
    
    /* Fill out the destination port */
    dest_addr.sa_data[0] = (port & 0xFF00) >> 8;
    dest_addr.sa_data[1] = (port & 0x00FF);
    
    /* Fill out the destination IP address */
    for (i = 0; i < 4; i++) {
        dest_addr.sa_data[i + 2] = ip_address[i];
    }
    
    /* Set the rest of the dest_addr struct to 0 */
    for (i = 6; i < 14; i++) {
        dest_addr.sa_data[i] = 0;
    }
    
    /* Attempt to make a connection with a remote socket */
    if (connect_to_socket(socket_, &dest_addr, sizeof(dest_addr)) != 
                                                        CC3000_SUCCESS) {
        close();
        return false;
    }

    return true;
}

/**
 * @brief Writes a single character to the socket
 *
 * @param[in] c the character to be written
 * @return the amount of data (in bytes) written
 */
size_t SFE_CC3000_Client::write(uint8_t c)
{
    return write(&c, 1);
}

/**
 * @brief Writes a string of characters to the socket
 *
 * @param[in] buf buffer of characters
 * @param[in] size the size (in bytes) of the buffer
 * @return the amount of data (in bytes) written
 */
size_t SFE_CC3000_Client::write(const uint8_t *buf, size_t size)
{
    /* If socket does not have a connection, return 0 */
    if (!connected()) {
        return 0;
    }
    
    /* Send buffer. Last parameter (flags) is not yet implemented by TI. */
    return send(socket_, buf, size, 0);
}

/**
 * @brief Connects to a remote server using UDP
 *
 * @param[in] hostname the address of the remote server
 * @param[in] port the receiving port of the server (default: 80)
 * @return True if connected to remote server. False otherwise.
 */
int SFE_CC3000_Client::connectUDP(    const char *hostname, 
                                    uint16_t port)
{
    IPAddress remote_ip;

    /* If CC3000 is not connected to a network, return false. */
    if (    !cc3000_->getInitStatus() || 
            !cc3000_->getConnectionStatus() || 
            !cc3000_->getDHCPStatus() ) {
        return false;
    }
    
    /* Perform a DNS lookup of the site */
    if (!cc3000_->dnsLookup(const_cast<char *>(hostname), &remote_ip)) {
        return false;
    }

    Serial.println(remote_ip);
    
    /* Connect to remote host using IP address */
    return connectUDP(remote_ip, port);
}

/**
 * @brief Connects to a remote server using UDP
 *
 * @param[in] IP address of the remote server
 * @param[in] port the receiving port of the server (default: 80)
 * @return True if connected to remote server. False otherwise.
 */
int SFE_CC3000_Client::connectUDP(    IPAddress ip_address, 
                                    uint16_t port)
{
    sockaddr dest_addr;
    int i;
                                    
    /* If CC3000 is not connected to a network, return false. */
    if (    !cc3000_->getInitStatus() || 
            !cc3000_->getConnectionStatus() || 
            !cc3000_->getDHCPStatus() ) {
        return false;
    }
    
    /* Create a socket */
    socket_ = socket(AF_INET, SOCK_STREAM, UDP);
    if (socket_ == -1) {
        return false;
    }
    
    /* Set address family to AF_INET (only one that works right now) */
    dest_addr.sa_family = AF_INET;
    
    /* Fill out the destination port */
    dest_addr.sa_data[0] = (port & 0xFF00) >> 8;
    dest_addr.sa_data[1] = (port & 0x00FF);
    
    /* Fill out the destination IP address */
    for (i = 0; i < 4; i++) {
        dest_addr.sa_data[i + 2] = ip_address[i];
    }
    
    /* Set the rest of the dest_addr struct to 0 */
    for (i = 6; i < 14; i++) {
        dest_addr.sa_data[i] = 0;
    }
    
    /* Attempt to make a connection with a remote socket */
    if (connect_to_socket(socket_, &dest_addr, sizeof(dest_addr)) != 
                                                        CC3000_SUCCESS) {
        close();
        return false;
    }

    return true;
}

/**
 * @brief Determines if data is available for reading
 *
 * @return True if socket contains data to be read. False otherwise.
 */
int SFE_CC3000_Client::available()
{
    fd_set readsds;
    timeval timeout;

    /* We need something in readsds to tell select() to watch read sockets */
    memset(&readsds, 1, sizeof(readsds));
    
    /* Minimum timeout for select() is 5ms */
    timeout.tv_sec = 0;
    timeout.tv_usec = 5000;
    
    /* Call select() to see if there is any data waiting */
    int ret = select(socket_ + 1, &readsds, NULL, NULL, &timeout);
    
    /* If select() returned anything greater than 0, there's data for us */
    if (ret > 0) {
        return true;
    } else {
        return false;
    }
}

/**
 * @brief reads 1 byte from the socket
 *
 * @return received data. -1 if no data is available
 */
int SFE_CC3000_Client::read()
{
    uint8_t b;
    
    if (recv(socket_, &b, 1, 0) > 0) {
        return b;
    } else {
        return -1;
    }
}

/**
 * @brief reads data from the socket
 *
 * @param[out] buf the buffer onto which the data is written
 * @param[in] the length (in bytes) of the buffer
 * @return number of bytes received. -1 if an error occurred
 */
int SFE_CC3000_Client::read(uint8_t *buf, size_t size)
{
    /* Note: Flags arg is not yet supported by TI library */
    return recv(socket_, buf, size, 0);
}

/**
 * @brief Closes the socket
 *
 * @return True if socket closed successfully. False otherwise.
 */
bool SFE_CC3000_Client::close()
{
    long ret;

    /* Attempt to close the socket and set connected state to false */
    ret = closesocket(socket_);
    socket_ = -1;
    g_socket_connected = false;
    return (ret == CC3000_SUCCESS) ? true : false;
}

/**
 * @brief Gets the status of the socket's connection
 *
 * @return True if socket is connected. False otherwise.
 */
uint8_t SFE_CC3000_Client::connected()
{
    /* If there is no socket, return false */
    if (socket_ == -1) {
        return false;
    }
    
    /* If the connection was closed and there is no data, return false */
    if ((!g_socket_connected) && (!available())) {
        return false;
    }
    
    return true;
}

/**
 * @brief NOT AVAILABLE IN CC3000 recv currently, necessary for Arduino Client implementation.
 *
 * @return -1 always
 */
 int SFE_CC3000_Client::peek(){
  return -1;
}

/**
 * @brief Discard any bytes that have been written to the client but not yet read.
 *
 * @return none
 */
void SFE_CC3000_Client::flush(){
  while(available())
    read();
}

/**
 * @brief Alias for close necessary for Arduino Client implementation.
 *
 * @return none
 */
void SFE_CC3000_Client::stop(){
  close();
}

SFE_CC3000_Client::operator bool()
{
  return (cc3000_->getInitStatus() || 
            cc3000_->getConnectionStatus() || 
            cc3000_->getDHCPStatus());
}
    