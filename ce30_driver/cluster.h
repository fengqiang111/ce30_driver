#ifndef __CLUSTER_H
#define __CLUSTER_H

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>

#include <ce30_driver/ce30_driver.h>
#include <ce30_driver/data_types.h>
#include <ce30_driver/export.h>


using namespace std;


namespace ce30_driver
{
class API cluster
{
    public:
        cluster();
        ~cluster();
        bool DBSCAN_kdtree_2steps(float eps_near, int min_sample_size_near,
                                  float eps_far, int min_sample_size_far,
                                  ce30_driver::PointCloud &cloud,
                                  vector<int> &label);
    private:
        void DBSCAN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                    std::vector<pcl::PointIndices>& cluster_indices,
                    float eps, int min_samples_size);
        float DBSCAN_2steps_boundary(void);
};

}

#endif
