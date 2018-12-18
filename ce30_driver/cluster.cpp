#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <vector>
#include <iterator>
#include "cluster.h"

using namespace std;
namespace ce30_driver
{
/**cluster constructor
  *@param none
  *@return none
  */
cluster::cluster()
{
}

cluster::~cluster()
{
}

/**��������
  *@param mode:CLUSTER_KD_TREE or CLUSTER_OC_TREE
  *@param eps_near: �������eps
  *@param min_sample_size_near: ����������С������
  *@param eps_far: Զ�����eps
  *@param min_sample_size_far: Զ��������С������
  *@param cloud:����������ݣ���������ĵ�������
  *@param label:������Ʒ����ǩ
  *@return true-success false-fail
  */
bool cluster::DBSCAN_2steps(cluster_mode mode, float eps_near, int min_sample_size_near,
                            float eps_far, int min_sample_size_far,
                            ce30_driver::PointCloud &cloud,
                            vector<int> &label)
{
    float boundary = DBSCAN_2steps_boundary();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_near(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_far(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::PointIndices> cluster_indices_near;
    std::vector<pcl::PointIndices> cluster_indices_far;
    int n_points_near = 0;
    int n_points_far = 0;
    vector<ce30_driver::Point>::iterator it;
    int point_label = 0;

    if (cloud.points.empty() || eps_near <= 0 || eps_far <= 0 ||
        min_sample_size_near <= 0 || min_sample_size_far <= 0)
    {
        return false;
    }

    for (auto& point : cloud.points)
    {
        if (point.x <= boundary)
        {
            n_points_near++;
            cloud_near->push_back({point.x, point.y, point.z});
        }
        else
        {
            n_points_far++;
            cloud_far->push_back({point.x, point.y, point.z});
        }
    }

    cloud.points.clear();
    label.clear();
    if (n_points_near != 0)
    {
        if (mode == CLUSTER_KD_TREE)
        {
            DBSCAN_kdtree(cloud_near, cluster_indices_near, eps_near, min_sample_size_near);
        }
        else
        {
            DBSCAN_octree(cloud_near, cluster_indices_near, eps_near, min_sample_size_near);
        }

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_near.begin();
             it != cluster_indices_near.end(); ++it)
        {
            for (std::vector<int>::const_iterator pit = it->indices.begin();
                 pit != it->indices.end(); ++pit)
            {
                auto point_xyz = cloud_near->points[*pit];
                ce30_driver::Point point(point_xyz.x, point_xyz.y, point_xyz.z);
                cloud.points.push_back(point);
                label.push_back(point_label);
            }
            point_label++;
        }
    }

    if (n_points_far != 0)
    {
        if (mode == CLUSTER_KD_TREE)
        {
            DBSCAN_kdtree(cloud_far, cluster_indices_far, eps_far, min_sample_size_far);
        }
        else
        {
            DBSCAN_octree(cloud_far, cluster_indices_far, eps_far, min_sample_size_far);
        }

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_far.begin();
             it != cluster_indices_far.end(); ++it)
        {
            for (std::vector<int>::const_iterator pit = it->indices.begin();
                 pit != it->indices.end(); ++pit)
            {
                auto point_xyz = cloud_far->points[*pit];
                pcl::PointXYZRGB pointXYZRGB;
                ce30_driver::Point point(point_xyz.x, point_xyz.y, point_xyz.z);
                cloud.points.push_back(point);
                label.push_back(point_label);
            }
            point_label++;
        }
    }

    return true;
}


/**DBSCAN kdtree���ƾ��࣬ʹ��ŷ�Ͼ�����ȡ����
  *@param cloud: ����һ֡�������ݣ�����pcl::PointCloud<pcl::PointXYZ>::Ptr
  *@param cluster_indices: ���������������std::vector<pcl::PointIndices>
  *@param eps: �����뾶
  *@param min_sample_size: ���������С����
  *@return none
  */
void cluster::DBSCAN_kdtree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                            std::vector<pcl::PointIndices>& cluster_indices,
                            float eps, int min_samples_size)
{
    if (cloud->empty() || eps <= 0 || min_samples_size <= 0)
    {
        return;
    }
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);

    //std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (eps);
    ec.setMinClusterSize (min_samples_size);
    ec.setMaxClusterSize (6400);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
}

/**DBSCAN octree���ƾ��࣬ʹ��ŷ�Ͼ�����ȡ����
  *@param cloud: ����һ֡�������ݣ�����pcl::PointCloud<pcl::PointXYZ>::Ptr
  *@param cluster_indices: ���������������std::vector<pcl::PointIndices>
  *@param eps: �����뾶
  *@param min_sample_size: ���������С����
  *@return none
  */
void cluster::DBSCAN_octree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                            std::vector<pcl::PointIndices>& cluster_indices,
                            float eps, int min_samples_size)
{
    if (cloud->empty() || eps <= 0 || min_samples_size <= 0)
    {
        return;
    }

    pcl::search::Octree<pcl::PointXYZ>::Ptr tree(new pcl::search::Octree<pcl::PointXYZ>(eps));
    tree->setInputCloud (cloud);

    //std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (eps);
    ec.setMinClusterSize (min_samples_size);
    ec.setMaxClusterSize (6400);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
}

/**�������෵�صı߽�
  *@param none
  *@return ���صı߽�
  */
float cluster::DBSCAN_2steps_boundary(void)
{
    return 0.6;
}
}
