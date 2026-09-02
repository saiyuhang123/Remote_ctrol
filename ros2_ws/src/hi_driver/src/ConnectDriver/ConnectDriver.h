#ifndef CONNECTDRIVER_H
#define CONNECTDRIVER_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <netdb.h> 
#include <vector>
#include <thread>
#include <functional>
#include <unistd.h>        /* System V */
#include <sys/ioctl.h>    /* BSD and Linux */
#include <cstdlib>

using namespace std;

 using RecvDataCallback =
    std::function<void(std::vector<unsigned char> &data,int size)>;
class ConnectDriver
{


  private:
    int sock;
    std::string address;
    int port;
    struct sockaddr_in server;
    int addrlen = 0;
    int len = 0;
    std::thread  listen_thread;
    RecvDataCallback recv_data_callback = NULL;
    int read_buf_len;
    volatile int count = 0;
    bool udpenable = false;

    bool openenable = false;

    bool udpopenenable = false;
    void Listen();

    


  public:
 
    ConnectDriver(bool udpenable);
    ~ConnectDriver();

    bool setup(string address, int port);
    bool Send(string data);
    bool Send(std::vector<unsigned char> data);
    bool Send(char *data, unsigned int len);

    string receive(int size = 4096);
    string read();
    void  Close();
    bool is_nodata(int rate);	
    bool clear_cnt();

    void setcallback(RecvDataCallback callback);
    bool is_open();


};

#endif
