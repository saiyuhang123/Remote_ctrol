#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <functional>
#include <memory>
#include <thread>
#include <atomic>

#include <boost/asio.hpp>

using boost::asio::ip::tcp;
using boost::asio::ip::udp;

using BoostRecvDataCallback = std::function<void(std::vector<unsigned char>& data, int size)>;

class NetworkDriver {
public:
    explicit NetworkDriver(bool udp_enable)
        : ioc_(),
          work_guard_(boost::asio::make_work_guard(ioc_)),
          tcp_socket_(ioc_),
          udp_socket_(ioc_),
          udp_enable_(udp_enable),
          is_connected_(false),
          running_(true)
    {
        raw_buffer_.resize(65535);
    }

    ~NetworkDriver()
    {
        Close();
    }

    // ================== 对外接口 ==================

    void StartThreads(int thread_count = 2)
    {
        for (int i = 0; i < thread_count; ++i) {
            threads_.emplace_back([this]() {
                ioc_.run();  // ★ 只用 run，不用 poll
            });
        }
    }

    void setCallback(BoostRecvDataCallback cb)
    {
        on_data_recv_ = std::move(cb);
    }

    bool setup(const std::string& host, int port)
    {
        if (!udp_enable_) {
            return setup_tcp(host, port);
        } else {
            return setup_udp(host, port);
        }
    }

    bool is_open() const
    {
        if (udp_enable_) {
            return udp_socket_.is_open();
        } else {
            return tcp_socket_.is_open() && is_connected_.load();
        }
    }

    bool Send(const std::vector<unsigned char>& data)
    {
        if (data.empty() || !is_open()) return false;

        auto send_buf =
            std::make_shared<std::vector<unsigned char>>(data);

        if (!udp_enable_) {
            boost::asio::async_write(
                tcp_socket_,
                boost::asio::buffer(*send_buf),
                [send_buf](boost::system::error_code ec, std::size_t) {
                    if (ec) {
                        std::cerr << "[TCP] send failed: "
                                  << ec.message() << std::endl;
                    }
                });
        } else {
            udp_socket_.async_send_to(
                boost::asio::buffer(*send_buf),
                udp_target_ep_,
                [send_buf](boost::system::error_code ec, std::size_t) {
                    if (ec) {
                        std::cerr << "[UDP] send failed: "
                                  << ec.message() << std::endl;
                    }
                });
        }
        return true;
    }

    bool Send(const char* data, unsigned int len)
    {
        if (!data || len == 0) return false;
        return Send(std::vector<unsigned char>(data, data + len));
    }

    void Close()
    {
        if (!running_.exchange(false)) return;

        is_connected_ = false;

        boost::system::error_code ec;

        if (tcp_socket_.is_open()) {
            tcp_socket_.shutdown(tcp::socket::shutdown_both, ec);
            tcp_socket_.close(ec);
        }

        if (udp_socket_.is_open()) {
            udp_socket_.close(ec);
        }

        work_guard_.reset();
        ioc_.stop();

        for (auto& t : threads_) {
            if (t.joinable()) t.join();
        }
        threads_.clear();
    }

private:
    // ================== TCP ==================

    bool setup_tcp(const std::string& host, int port)
    {
        is_connected_ = false;

        try {
            tcp::resolver resolver(ioc_);
            auto endpoints =
                resolver.resolve(host, std::to_string(port));

            boost::asio::async_connect(
                tcp_socket_,
                endpoints,
                [this](boost::system::error_code ec, tcp::endpoint ep) {
                    if (!ec && running_) {
                        is_connected_ = true;
                        std::cout << "[TCP] connected to "
                                  << ep << std::endl;
                        do_tcp_read();
                    } else {
                        is_connected_ = false;
                        std::cerr << "[TCP] connect failed: "
                                  << ec.message() << std::endl;
                    }
                });
             for(int i = 0; i < 2; i++)
            {
                if(is_open())
                    return true;
                std::this_thread::sleep_for(std::chrono::seconds(1));  
                
            }    
            return false;
        } catch (std::exception& e) {
            std::cerr << "[TCP] exception: " << e.what() << std::endl;
            return false;
        }
    }

    void do_tcp_read()
    {
        tcp_socket_.async_read_some(
            boost::asio::buffer(raw_buffer_),
            [this](boost::system::error_code ec, std::size_t length) {
                if (!running_) return;

                if (!ec) {
                    dispatch_data(length);
                    do_tcp_read();
                } else {
                    is_connected_ = false;
                    boost::system::error_code ignore;
                    tcp_socket_.close(ignore);
                }
            });
    }

    // ================== UDP ==================

    bool setup_udp(const std::string& host, int port)
    {
        try {
            udp_socket_.open(udp::v4());

            udp_target_ep_ =
                *udp::resolver(ioc_)
                     .resolve(udp::v4(), host, std::to_string(port))
                     .begin();

            do_udp_receive();

            is_connected_ = true;
            return true;
        } catch (std::exception& e) {
            std::cerr << "[UDP] exception: " << e.what() << std::endl;
            return false;
        }
    }

    void do_udp_receive()
    {
        udp_socket_.async_receive_from(
            boost::asio::buffer(raw_buffer_),
            udp_sender_ep_,  // ★ 接收端点与发送目标分离
            [this](boost::system::error_code ec, std::size_t length) {
                if (!running_) return;

                if (!ec) {
                    dispatch_data(length);
                    do_udp_receive();
                }
            });
    }

    // ================== 数据派发（线程安全） ==================

    void dispatch_data(std::size_t length)
    {
        if (!on_data_recv_) return;

        // ★ 拷贝一份，避免 buffer 被覆盖
        std::vector<unsigned char> data(
            raw_buffer_.begin(),
            raw_buffer_.begin() + length);

        on_data_recv_(data, static_cast<int>(length));
    }

private:
    boost::asio::io_context ioc_;
    boost::asio::executor_work_guard<
        boost::asio::io_context::executor_type>
        work_guard_;

    tcp::socket tcp_socket_;
    udp::socket udp_socket_;

    udp::endpoint udp_target_ep_;  // 发送目标
    udp::endpoint udp_sender_ep_;  // 接收来源

    std::vector<unsigned char> raw_buffer_;

    BoostRecvDataCallback on_data_recv_;

    std::atomic<bool> is_connected_;
    std::atomic<bool> running_;

    bool udp_enable_;

    std::vector<std::thread> threads_;
};
