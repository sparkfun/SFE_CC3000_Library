/**
 * @file    SFE_CC3000_Client.h
 * @brief   Library for the SparkFun CC3000 shield and breakout boards
 * @author  Shawn Hymel (SparkFun Electronics)
 * 
 * @copyright   This code is public domain but you buy me a beer if you use
 * this and we meet someday (Beerware license).
 * 
 * The client library provides functions to connect to servers using sockets.
 */
 
#ifndef SFE_CC3000_CLIENT_H
#define SFE_CC3000_CLIENT_H

#include <Arduino.h>

#include "SFE_CC3000.h"
#include "utility/socket.h"                 // Needed for socket communications
#include "Client.h"

/* Constants for IP protocol types */
#define TCP     IPPROTO_TCP
#define UDP     IPPROTO_UDP

/* CC3000 Client class */
class SFE_CC3000_Client : public Client {
public:
    SFE_CC3000_Client(SFE_CC3000 &cc3000);
    ~SFE_CC3000_Client();

    int connect(   const char *hostname, 
                    uint16_t port = 80);
    int connect(   IPAddress ip_address, 
                    uint16_t port = 80);
    int connectUDP(   const char *hostname, 
                    uint16_t port = 80);
    int connectUDP(   IPAddress ip_address, 
                    uint16_t port = 80);

    virtual size_t write(uint8_t c);
    virtual size_t write(const uint8_t *buf, size_t size);
    int available();
    int read();
    int read(uint8_t *buf, size_t size);
    bool close();
    uint8_t connected();

    int peek();
    void flush();
    void stop();

    operator bool();

private:
    SFE_CC3000 *cc3000_;
    int32_t socket_;
};

#endif