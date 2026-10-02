#ifndef TMNAV_EIGEN_CONFIG_H
#define TMNAV_EIGEN_CONFIG_H
// Must precede every Eigen include in this library. No heap, SIMD, or threads.
#define EIGEN_NO_MALLOC
#define EIGEN_DONT_VECTORIZE
#define EIGEN_DONT_PARALLELIZE
#define EIGEN_UNROLLING_LIMIT 0
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#endif
