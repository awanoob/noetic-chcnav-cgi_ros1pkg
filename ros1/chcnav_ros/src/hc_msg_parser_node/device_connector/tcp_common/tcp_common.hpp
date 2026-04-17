#ifndef __HC_TCP_COMMON_HPP_
#define __HC_TCP_COMMON_HPP_

#include "device_connector.hpp"

#include <cstring>
#include <iostream>
#include <string>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <netinet/tcp.h>

#define TCP_MAX_CONNECT_NUM 300

class tcp_common : public hc__device_connector
{
private:
    int status = -1;                // 连接状态。 -1 未连接，1 已连接
    std::string host;               // ip
    int port;                       // 端口号
    int socketfd;                   // socket fd
    struct sockaddr_in client_addr; // 客户端信息结构体

    bool setupKeepAlive() {
        int keepalive = 1;    // 启用保活机制
        int keepidle = 5;    // 空闲5秒后开始发送保活包
        int keepintvl = 5;    // 发送保活包的间隔为5秒
        int keepcnt = 3;      // 最多发送3次保活包

        if (setsockopt(socketfd, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive)) < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("tcp_common"), "设置SO_KEEPALIVE失败");
            return false;
        }

        // 设置保活空闲时间
        if (setsockopt(socketfd, IPPROTO_TCP, TCP_KEEPIDLE, &keepidle, sizeof(keepidle)) < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("tcp_common"), "设置TCP_KEEPIDLE失败");
            return false;
        }

        // 设置保活探测包发送间隔
        if (setsockopt(socketfd, IPPROTO_TCP, TCP_KEEPINTVL, &keepintvl, sizeof(keepintvl)) < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("tcp_common"), "设置TCP_KEEPINTVL失败");
            return false;
        }

        // 设置保活探测次数
        if (setsockopt(socketfd, IPPROTO_TCP, TCP_KEEPCNT, &keepcnt, sizeof(keepcnt)) < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("tcp_common"), "设置TCP_KEEPCNT失败");
            return false;
        }

        return true;
    }

public:
    tcp_common(std::string host, int port) : hc__device_connector()
    {
        this->host = host;
        this->port = port;

        this->client_addr.sin_family = AF_INET;
        this->client_addr.sin_port = htons(this->port);
        this->client_addr.sin_addr.s_addr = inet_addr(this->host.c_str());
    }

    int connect() override {
        if (this->status != -1)
            this->disconnect();

        this->socketfd = socket(AF_INET, SOCK_STREAM, 0);
        if (this->socketfd < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("tcp_common"), "创建socket失败");
            return -1;
        }

        // 设置TCP保活机制
        if (!setupKeepAlive()) {
            RCLCPP_WARN(rclcpp::get_logger("tcp_common"), "TCP保活机制设置失败");
        }

        if (::connect(this->socketfd, (struct sockaddr *)&this->client_addr, sizeof(this->client_addr)) == -1) {
            this->status = -1;
            RCLCPP_ERROR(rclcpp::get_logger("tcp_common"), 
                "连接服务器失败: %s:%d", this->host.c_str(), this->port);
            return -1;
        }

        this->status = 1;
        RCLCPP_INFO(rclcpp::get_logger("tcp_common"), 
            "成功连接到服务器: %s:%d", this->host.c_str(), this->port);
        return 0;
    }

    void judge_connect_state()
    {
        if(this->status != 1)
    	{
    	    int reconnect_num = 0;
    	    while(reconnect_num < TCP_MAX_CONNECT_NUM)
    	    {
    	    	if(connect() == 0)
    	    	{
    	    	    fprintf(stdout, "tcp reconnect success!\n");
    	    	    break; 
    	    	}
    	    	
    	    	reconnect_num++;
    	    	fprintf(stderr, "tcp reconnect fail!\n");
    	    	usleep(100 * 1000);
    	    }
    	}
    }

    int write(char *data, unsigned int len)
    {
        while (this->status != 1)
        {
            fprintf(stderr, "tcp disconnect, reconnect!\n");
            this->connect();
            sleep(1);
        }

        int write_bytes = ::write(this->socketfd, data, len);
        if (write_bytes == -1)
        {
            printf("socker [%d] send failed\n", this->socketfd);
            this->disconnect();
            return -1;
        }

        return write_bytes;
    }

    int read(char *data, unsigned int maxsize)
    {
        while (this->status != 1)
        {
            fprintf(stderr, "tcp disconnect, reconnect!\n");
            this->connect();
            sleep(1);
        }

        int read_bytes = ::read(this->socketfd, data, maxsize);
        if (read_bytes <= 0)
        {
            if (errno == EWOULDBLOCK) // nothing sended by client sock
            {
                errno = 0;
            }
            else
            {
                printf("socker [%d] read failed\n", this->socketfd);
                this->disconnect();
                return -1;
            }
        }

        return read_bytes;
    }

    int disconnect(void)
    {
        close(socketfd);
        this->status = -1;

        return 0;
    }
};

#endif
