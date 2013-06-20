#ifndef _MINIPCL_H_
#define _MINIPCL_H_

#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>

void mpcl_compute_normals(pcl::PointCloud<pcl::PointXYZ> &cloud,
                          int ksearch,
                          double searchRadius,
                          pcl::PointCloud<pcl::PointNormal> &out);

void mpcl_sacnormal_set_axis(pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::PointNormal> &sac,
                             double ax, double ay, double az);

void mpcl_extract(pcl::PointCloud<pcl::PointXYZ> &incloud,
                  pcl::PointCloud<pcl::PointXYZ> &outcloud,
                  pcl::PointIndices *indices,
                  bool negative);

#endif
