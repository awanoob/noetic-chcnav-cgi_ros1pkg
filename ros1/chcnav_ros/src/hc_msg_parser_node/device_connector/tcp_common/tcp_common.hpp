#ifndef __HC_TCP_COMMON_HPP_
#define __HC_TCP_COMMON_HPP_

#include "device_connector.hpp"

#include <cerrno>
#include <cstring>
#include <iostream>
#include <string>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#define TCP_MAX_CONNECT_NUM 300
#define TCP_RECONNECT_INTERVAL_US (100 * 1000)

class tcp_common : public hc__device_connector
{
private:
    int status = -1;                // 连接状态。 -1 未连接，1 已连接
    std::string host;               // ip
    int port;                       // 端口号
    int socketfd = -1;              // socket fd
    bool host_valid = false;
    bool connect_error_logged = false;
    unsigned int connection_generation = 0;
    struct sockaddr_in client_addr; // 客户端信息结构体

    void close_socket()
    {
        if (this->socketfd >= 0)
        {
            close(this->socketfd);
            this->socketfd = -1;
        }
    }

    void mark_disconnected()
    {
        close_socket();
        this->status = -1;
    }

    bool setupKeepAlive()
    {
        int keepalive = 1; // 启用保活机制
        int keepidle = 5;  // 空闲5秒后开始发送保活包
        int keepintvl = 5; // 发送保活包的间隔为5秒
        int keepcnt = 3;   // 最多发送3次保活包

        if (setsockopt(this->socketfd, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive)) < 0)
        {
            ROS_ERROR("设置SO_KEEPALIVE失败");
            return false;
        }

        if (setsockopt(this->socketfd, IPPROTO_TCP, TCP_KEEPIDLE, &keepidle, sizeof(keepidle)) < 0)
        {
            ROS_ERROR("设置TCP_KEEPIDLE失败");
            return false;
        }

        if (setsockopt(this->socketfd, IPPROTO_TCP, TCP_KEEPINTVL, &keepintvl, sizeof(keepintvl)) < 0)
        {
            ROS_ERROR("设置TCP_KEEPINTVL失败");
            return false;
        }

        if (setsockopt(this->socketfd, IPPROTO_TCP, TCP_KEEPCNT, &keepcnt, sizeof(keepcnt)) < 0)
        {
            ROS_ERROR("设置TCP_KEEPCNT失败");
            return false;
        }

        return true;
    }

public:
    tcp_common(std::string host, int port) : hc__device_connector()
    {
        this->host = host;
        this->port = port;

        std::memset(&this->client_addr, 0, sizeof(this->client_addr));
        this->client_addr.sin_family = AF_INET;
        this->client_addr.sin_port = htons(this->port);
        this->host_valid = inet_pton(AF_INET, this->host.c_str(), &this->client_addr.sin_addr) == 1;
    }

    ~tcp_common()
    {
        this->disconnect();
    }

    int connect() override
    {
        if (!this->host_valid)
        {
            if (!this->connect_error_logged)
            {
                ROS_ERROR("invalid IPv4 address: %s", this->host.c_str());
                this->connect_error_logged = true;
            }
            this->mark_disconnected();
            return -1;
        }

        if (this->status != -1)
            this->disconnect();

        this->socketfd = socket(AF_INET, SOCK_STREAM, 0);
        if (this->socketfd < 0)
        {
            if (!this->connect_error_logged)
            {
                ROS_ERROR("create socket failed errno=%d", errno);
                this->connect_error_logged = true;
            }
            this->mark_disconnected();
            return -1;
        }

        if (!setupKeepAlive())
            ROS_WARN("TCP keepalive setup failed");

        if (::connect(this->socketfd, (struct sockaddr *)&this->client_addr, sizeof(this->client_addr)) == -1)
        {
            if (!this->connect_error_logged)
            {
                ROS_ERROR("connect server failed: %s:%d errno=%d", this->host.c_str(), this->port, errno);
                this->connect_error_logged = true;
            }
            this->mark_disconnected();
            return -1;
        }

        this->status = 1;
        this->connection_generation++;
        this->connect_error_logged = false;
        ROS_INFO("connect server success: %s:%d", this->host.c_str(), this->port);
        return 0;
    }

    void judge_connect_state() override
    {
        if (this->status != 1)
        {
            int reconnect_num = 0;
            while (reconnect_num < TCP_MAX_CONNECT_NUM && this->status != 1)
            {
                if (connect() == 0)
                {
                    fprintf(stdout, "tcp reconnect success!\n");
                    break;
                }

                reconnect_num++;
                if (reconnect_num < TCP_MAX_CONNECT_NUM)
                    usleep(TCP_RECONNECT_INTERVAL_US);
            }
        }
    }

    unsigned int get_connection_generation() const override
    {
        return this->connection_generation;
    }

    int write(char *data, unsigned int len) override
    {
        if (this->status != 1 || this->socketfd < 0)
            return -1;

        unsigned int total_written = 0;
        while (total_written < len)
        {
            ssize_t write_bytes = ::write(this->socketfd, data + total_written, len - total_written);
            if (write_bytes > 0)
            {
                total_written += static_cast<unsigned int>(write_bytes);
                continue;
            }

            if (write_bytes < 0 && errno == EINTR)
                continue;

            if (write_bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
                continue;

            ROS_ERROR("socket[%d] send failed errno=%d", this->socketfd, errno);
            this->mark_disconnected();
            return -1;
        }

        return static_cast<int>(total_written);
    }

    int read(char *data, unsigned int maxsize) override
    {
        if (this->status != 1 || this->socketfd < 0)
            return -1;

        while (1)
        {
            ssize_t read_bytes = ::read(this->socketfd, data, maxsize);
            if (read_bytes > 0)
                return static_cast<int>(read_bytes);

            if (read_bytes == 0)
            {
                ROS_WARN("TCP peer closed connection: %s:%d", this->host.c_str(), this->port);
                this->mark_disconnected();
                return -1;
            }

            if (errno == EINTR)
                continue;

            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                errno = 0;
                return 0;
            }

            ROS_ERROR("socket[%d] read failed errno=%d", this->socketfd, errno);
            this->mark_disconnected();
            return -1;
        }
    }

    int disconnect(void) override
    {
        this->mark_disconnected();
        return 0;
    }
};

#endif
