#include <iostream>
#include <ce30_driver/ce30_driver.h>
#include <math.h>
#include <vector>

using namespace std;
using namespace ce30_driver;
#if 0
/**callback function
  *@param cloud-一帧点云数据
  *@return none
  */
void DataReceiveCB(shared_ptr<PointCloud> cloud)
{
    for (Point& point : cloud->points)
    {
        cout << point.x << " " << point.y << " " << point.z << endl;
    }
}

/**cluster callback function
  *@param cloud-一帧点云数据
  *@return none
  */
void cluster_callback(shared_ptr<PointCloud> cloud)
{
    int i;
    vector<int> labels;
    PointCloud point_cloud;
    cluster cluster_mgr;

    point_cloud.points = cloud->points;
    cluster_mgr.DBSCAN_2steps(CLUSTER_KD_TREE, 0.05, 40, 0.30, 20, point_cloud, labels);


    for (i = 0; i < labels.size(); ++i)
    {
        cout << "<x,y,z> = " << point_cloud.points[i].x << " " <<
                                point_cloud.points[i].y << " " <<
                                point_cloud.points[i].z <<
                                " labels = " << labels[i] << endl;
    }
}

int main()
{
    UDPServer server;
    server.RegisterCallback(cluster_callback);
    if (!server.Start())
    {
        return -1;
    }
    while (true)d<
    {
        server.SpinOnce();
    }
}
#else
int main()
{
    int i;
    UDPSocket socket;
    PointCloud point_cloud;
    //cluster cluster_mgr;
    //vector<int> labels;
    Scan distance_scan;
    Scan gray_scan;

    if (!Connect(socket))
    {
        return -1;
    }

    // get CE30-D version
    VersionRequestPacket version_request;
    if (!SendPacket(version_request, socket))
    {
        return -1;
    }

    VersionResponsePacket version_response;
    if (!GetPacket(version_response, socket))
    {
        return -1;
    }

    cout << "CE30-D version: " << version_response.GetVersionString() << endl;

	GetIDRequestPacket ID_request;

    if (!SendPacket(ID_request, socket))
    {
        return -1;
    }

    GetIDResponsePacket ID_response;
    if (!GetPacket(ID_response, socket))
    {
        return -1;
    }
    std::vector<unsigned char> id = ID_response.ID();
    
    cout << "id: ";
    for (auto i = 0; i < id.size(); ++i)
    {
        cout << id[i];
    }
 
    cout << "\n";

    // start get distance frame
    StartRequestPacket start_request;
    if (!SendPacket(start_request, socket))
    {
        return -1;
    }
     

    Packet packet;
    while (true)
    {
        //labels.clear();
        point_cloud.points.clear();
        gray_scan.Reset();
        distance_scan.Reset();
        while (!distance_scan.Ready() && !gray_scan.Ready())
        {
            if (!GetPacket(packet, socket))
            {
                continue;
            }
            // parse packet3
            unique_ptr<ParsedPacket> parsed = packet.Parse();
            if (parsed)
            {
                if (parsed->grey_image)
                {
                    gray_scan.AddColumnsFromPacket(*parsed);
                }
                else
                {
                    distance_scan.AddColumnsFromPacket(*parsed);
                }
            }
        }

        if (distance_scan.Ready())
        {
            for (int x = 0; x < distance_scan.Width(); ++x)
            {
                for (int y = 0; y < distance_scan.Height(); ++y)
                {
                    auto channel = distance_scan.at(x, y);
                    Point p = channel.point();
                    if (sqrt(p.x * p.x + p.y * p.y) < 0.1f)
                    {
                        continue;
                    }
                    point_cloud.points.push_back(p);
                }
            }

            // feed point cloud to cluster
//            cout << "be point size = " << point_cloud.points.size() << ", label size = " << labels.size() << endl;
//            cluster_mgr.DBSCAN_2steps(CLUSTER_KD_TREE, 0.05, 40, 0.30, 20, point_cloud, labels);
//            cout << "af point size = " << point_cloud.points.size() << ", label size = " << labels.size() << endl;
          cout << point_cloud.points.size() << endl;
/*
          for (i = 0; i < point_cloud.points.size(); ++i)
          {
                cout << "<x,y,z> = " << point_cloud.points[i].x << " " <<
                                        point_cloud.points[i].y << " " <<
                                        point_cloud.points[i].z << endl;
          }

*/

        }
    }
}

#endif
