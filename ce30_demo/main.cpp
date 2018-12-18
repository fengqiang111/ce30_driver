#include <iostream>
#include <map>
#include <ce30_driver/ce30_driver.h>

using namespace std;
using namespace ce30_driver;
#if 1
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

/**获取一个cluster中的中值数据
  *@param point_cloud-一帧点云数据
  *@param labels
  *@param min_distance
  *@return none
  */
bool get_max_distance(PointCloud &point_cloud, vector<int> &labels,
                      vector<float> &min_distances)
{
    int i;
    int label;
    float distance;

    map<int, float> map_labels_max_distance;
    map<int, float> map_labels_min_distance;

    map_labels_max_distance.clear();
    map_labels_min_distance.clear();

    for (i = 0; i < point_cloud.points.size(); ++i)
    {
        label = labels[i];
        distance = point_cloud.points[i].x;

        if (map_labels_max_distance.count(label))
        {
            if (distance > map_labels_max_distance[label])
            {
                map_labels_max_distance[label] = distance;
            }
        }
        else
        {
            map_labels_max_distance[label] = distance;
        }


        if (map_labels_min_distance.count(label))
        {
            if (distance < map_labels_min_distance[label])
            {
                map_labels_min_distance[label] = distance;
            }
        }
        else
        {
            map_labels_min_distance[label] = distance;
        }
    }

    for (i = 0; i < map_labels_max_distance.size(); ++i)
    {
        float max_distance;
        float min_distance;
        max_distance = map_labels_max_distance[i];
        min_distance = map_labels_min_distance[i];
        distance = min_distance + (max_distance - min_distance) * 0.75;
        if (distance < 0.54)
        {
            min_distances.push_back(distance);
        }
    }

    return true;
}

/**获取一个cluster中的均值数据
  *@param point_cloud-一帧点云数据
  *@param labels
  *@param min_distance
  *@return none
  */
bool get_average_distance(PointCloud &point_cloud, vector<int> &labels,
                          vector<float> &min_distances)
{
    int i;
    float distance;

    map<int, float> map_labels_sum;
    map<int, int> map_labels_num;

    map_labels_num.clear();
    map_labels_sum.clear();

    // 统计label下点的距离和以及个数
    for (i = 0; i < point_cloud.points.size(); ++i)
    {
        map_labels_sum[labels[i]] += point_cloud.points[i].x;
        map_labels_num[labels[i]] += 1;

    }

    // 输出label的均值距离
    for (i = 0; i < map_labels_sum.size(); ++i)
    {
        distance = map_labels_sum[i] / map_labels_num[i];
        if (distance < 0.54)
        {
            min_distances.push_back(distance);
        }
    }

    return true;
}

/**cluster callback function
  *@param cloud-一帧点云数据
  *@return none
  */
void cluster_callback(shared_ptr<PointCloud> cloud)
{
    vector<int> labels;
    vector<float> min_distances;
    PointCloud point_cloud;
    cluster cluster_mgr;
    static int count = 0;

    point_cloud.points.clear();
    point_cloud.points = cloud->points;
    cluster_mgr.DBSCAN_2steps(CLUSTER_KD_TREE, 0.01375, 30, 0.055, 30, point_cloud, labels);

    get_average_distance(point_cloud, labels, min_distances);

    cout << "***********************" << endl;
    for (vector<float>::iterator it = min_distances.begin(); it < min_distances.end(); it++)
    {
        cout << count << " = " << *it << endl;
    }
    cout << "***********************" << endl << endl;


}

int main()
{
    UDPServer server;
    server.RegisterCallback(cluster_callback);
    if (!server.Start())
    {
        return -1;
    }
    while (true)
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
    cluster cluster_mgr;
    vector<int> labels;
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

    // start get distance frame
    StartRequestPacket start_request;
    if (!SendPacket(start_request, socket))
    {
        return -1;
    }

    Packet packet;
    while (true)
    {
        labels.clear();
        point_cloud.points.clear();
        gray_scan.Reset();
        distance_scan.Reset();
        while (!distance_scan.Ready() && !gray_scan.Ready())
        {
            if (!GetPacket(packet, socket))
            {
                continue;
            }
            // parse packet
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
            cout << "be point size = " << point_cloud.points.size() << ", label size = " << labels.size() << endl;
            cluster_mgr.DBSCAN_2steps(CLUSTER_KD_TREE, 0.05, 40, 0.30, 20, point_cloud, labels);
            cout << "af point size = " << point_cloud.points.size() << ", label size = " << labels.size() << endl;
    //      for (i = 0; i < labels.size(); ++i)
    //      {
    //            cout << "<x,y,z> = " << point_cloud.points[i].x << " " <<
    //                                    point_cloud.points[i].y << " " <<
    //                                    point_cloud.points[i].z <<
    //                                    " labels = " << labels[i] << endl;
    //       }



        }
    }
}

#endif
