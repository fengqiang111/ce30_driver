#include "udp_server.h"
#include <stdexcept>
#include "utils.h"
#include <chrono>
#include <iostream>
#include <cmath>

using namespace std;

namespace ce30_driver
{
bool UDPServer::instance_running_ = false;

/** UPDServer constructor, kill_signal_=false, ip_=192.168.1.80, port_=2368
  *@param none
  *@return none
  */
UDPServer::UDPServer() : kill_signal_(false), ip_("192.168.1.80"), port_(2368)
{
    if (instance_running_)
    {
        throw runtime_error("Only one instance of UDPServer is allowed!");
    }
    instance_running_ = true;
}

/** UPDServer destructor kill_signal_ = true
  *@param none
  *@return none
  */
UDPServer::~UDPServer()
{
    kill_signal_ = true;
    if (thread_)
    {
        if (thread_->joinable())
        {
            thread_->join();
        }
    }
    StopRunning(*socket_);
}

/**创建udp socket，获取版本号并启动数据，创建数据接收线程
  *@param none
  *@return true - success, false - fail
  */
bool UDPServer::Start()
{
    if (!socket_)
    {
        socket_.reset(new UDPSocket(ip_, port_));
    }
    if (!Connect(*socket_))
    {
        return false;
    }
    string version;
    if (!GetVersion(version, *socket_))
    {
        return false;
    }
    if (!StartRunning(*socket_))
    {
        return false;
    }
    thread_.reset(new thread(&UDPServer::DataReceiveThread, this));
    return true;
}

/**
  *@param pointcloud
  *@return none
  */
void UDPServer::OnScanReceived(std::shared_ptr<PointCloud> pointcloud)
{

}

/**数据接收线程,接收到的数据放到scan_里面，通过条件变量通知阻塞在数据获取的线程
  *@param none
  *@return none
  */
void UDPServer::DataReceiveThread()
{
    while (true)
    {
        // signal_mutex_.lock();
        auto kill_signal = kill_signal_;
        // signal_mutex_.unlock();
        if (kill_signal)
        {
            return;
        }

        static Packet packet;
        static Scan scan;
        while (!scan.Ready())
        {
            if (GetPacket(packet, *socket_, true))
            {
                auto parsed = packet.Parse();
                if (parsed)
                {
                    scan.AddColumnsFromPacket(*parsed);
                }
            }
        }
        scan_mutex_.lock();
        scan_ = scan;
        condition_.notify_all();
        scan_mutex_.unlock();
        scan.Reset();
    }
}

/**获取一帧(20 * 320)数据
  *@param millisec: 超时时间，毫秒
  *@return none
  */
void UDPServer::SpinOnce(const int& millisec)
{
    static Scan scan;
    std::unique_lock<std::mutex> lock(scan_mutex_);
    bool timeout = (condition_.wait_for(lock,
                                        millisec * std::chrono::microseconds(millisec)) !=
                    cv_status::no_timeout);
    if (!timeout)
    {
        scan = scan_;
    }
    lock.unlock();

    if (timeout)
    {
        return;
    }

    shared_ptr<PointCloud> cloud(new PointCloud);
    cloud->points.reserve(scan.Width() * scan.Height());
    if (scan.Ready())
    {
        for (int x = 0; x < scan.Width(); ++x)
        {
            for (int y = 0; y < scan.Height(); ++y)
            {
                Point p = scan.at(x, y).point();
                if (sqrt(p.x * p.x + p.y * p.y) < 0.30f)
                {
                    continue;
                }
                cloud->points.push_back(p);
            }
        }
    }

    if (callback_)
    {
        (*callback_)(cloud);
    }
    else
    {
        OnScanReceived(cloud);
    }
}

/**注册对一帧点云数据的处理函数
  *@param callback-回调函数，在接收完一帧点云数据后被调用
  *@return none
  */
void UDPServer::RegisterCallback(std::function<void (std::shared_ptr<PointCloud>)> callback)
{
    callback_.reset(new std::function<void(std::shared_ptr<PointCloud>)>(callback));
}
}
