#ifndef POINTXYZIRL_CLASS
#define POINTXYZIRL_CLASS

#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>

// Algorithms we want this type to work with
#include <pcl/kdtree/impl/kdtree_flann.hpp>

namespace pcl {

  struct POINTXYZIRL {
    PCL_ADD_POINT4D                  // Macro quad-word XYZ
    float intensity;                 // Laser intensity
    uint16_t ring;                   // Laser ring number
    uint16_t label;                  // Label number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Ensure proper alignment
  } EIGEN_ALIGN16;

}  // end namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(POINTXYZIRL,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring))
    (uint16_t, label, label))

typedef pcl::PointCloud<pcl::POINTXYZIRL> PointCloudXYZIRL;

#endif
