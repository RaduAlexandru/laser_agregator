/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011, 2012 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id: data_base.h 1554 2011-06-14 22:11:17Z jack.oquin $
 */

/** \file
 *
 *  Point Cloud Library point structures for Velodyne data.
 *
 *  @author Jesse Vera
 *  @author Jack O'Quin
 *  @author Piyush Khandelwal
 */

#pragma once

#include <pcl/point_types.h>


/** Euclidean Velodyne coordinate, including intensity and ring number. */
struct PointXYZIR
{
  PCL_ADD_POINT4D;                    // quad-word XYZ
  float    intensity;                 ///< laser intensity reading
  uint16_t ring;                      ///< laser ring number
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
} EIGEN_ALIGN16;

/** Euclidean Velodyne coordinate, including intensity, distance and ring number. */
struct PointXYZIDR
{
   PCL_ADD_POINT4D;                    // quad-word XYZ
   float    intensity;                 ///< laser intensity reading
   float    distance;                  ///< distance of point to sensor
   uint16_t ring;                      ///< laser ring number
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
} EIGEN_ALIGN16;



POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIDR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (float, distance, distance)
                                  (uint16_t, ring, ring))
