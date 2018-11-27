#include "timed_udp_socket.h"
#include <boost/bind.hpp>
#include <iostream>

using namespace boost::asio::ip;

/// @cond DO_NOT_DOCUMENT_THIS
namespace ce30_driver
{

/** TimedUDPSocket constructor
  *@param listen_endpoint:listen_endpoint_=listen_endpoint
  *@return none
  */
TimedUDPSocket::TimedUDPSocket(const udp::endpoint& listen_endpoint)
    : socket_(io_service_, udp::endpoint(udp::v4(), listen_endpoint.port())),
      deadline_(io_service_),
      listen_endpoint_(listen_endpoint)
{
    // positive or negative infinity
    deadline_.expires_at(boost::posix_time::pos_infin);
    CheckDeadline();
}

/**
  *@param buffer
  *@param timeout
  *@param ec
  *@return none
  */
std::size_t TimedUDPSocket::Receive(const boost::asio::mutable_buffer &buffer,
                                    boost::posix_time::time_duration timeout,
                                    boost::system::error_code &ec)
{
    deadline_.expires_from_now(timeout);
    ec = boost::asio::error::would_block;
    std::size_t length = 0;
    socket_.async_receive(boost::asio::buffer(buffer),
                          boost::bind(&TimedUDPSocket::HandleReceive, _1, _2, &ec, &length));
    do
    {
        io_service_.run_one();
    }
    while (ec == boost::asio::error::would_block);
    return length;
}

/**
  *@param none
  *@return none
  */
void TimedUDPSocket::CheckDeadline()
{
    // expires_at()返回定时器当前的结束时间
    if (deadline_.expires_at() <=
        boost::asio::deadline_timer::traits_type::now())
    {
        socket_.cancel();
        deadline_.expires_at(boost::posix_time::pos_infin);
    }
    deadline_.async_wait(boost::bind(&TimedUDPSocket::CheckDeadline, this));
}

/**
  *@param ec:
  *@param length:
  *@param out_ec:
  *@param out_length:
  *@return none
  */
void TimedUDPSocket::HandleReceive(const boost::system::error_code &ec,
                                   std::size_t length,
                                   boost::system::error_code *out_ec,
                                   std::size_t *out_length)
{
    *out_ec = ec;
    *out_length = length;
}

/**
  *@param none
  *@return none
  */
udp::socket& TimedUDPSocket::GetSocket()
{
    return socket_;
}

/**
  *@param none
  *@return none
  */
udp::endpoint TimedUDPSocket::GetEndpoint()
{
    return listen_endpoint_;
}
}

/// @endcond
