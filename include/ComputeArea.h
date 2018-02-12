/*
 * Copyright (C) 2018
 * Authors: Nicolò Genesio
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#ifndef PCL_PLANE_PROJECTION_COMPUTEAREA_H
#define PCL_PLANE_PROJECTION_COMPUTEAREA_H


#include <pcl/io/pcd_io.h>
#include <math.h>


double ComputeArea(pcl::PointXYZRGB p1,pcl::PointXYZRGB p2,pcl::PointXYZRGB p3);

#endif //PCL_PLANE_PROJECTION_COMPUTEAREA_H
