/*
 * Copyright (C) 2018
 * Authors: Nicolò Genesio
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <iostream>
#include <ComputeArea.h>

using namespace std;
using namespace pcl;


double ComputeArea(pcl::PointXYZRGB p0,pcl::PointXYZRGB p1,pcl::PointXYZRGB p2){

    float area=0.0;
    Eigen::Vector3f v,w,tot;

    v[0]=(p1.x-p0.x);
    v[1]=(p1.y-p0.y);
    v[2]=(p1.z-p0.z);
    w[0]=(p2.x-p0.x);
    w[1]=(p2.y-p0.y);
    w[2]=(p2.z-p0.z);
    tot=v.cross(w);
    /*std::vector<float> prodvec;
    prodvec.resize(3);
    v.x=(p1.x-p0.x);
    v.y=(p1.y-p0.y);
    v.z=(p1.z-p0.z);
    w.x=(p2.x-p0.x);
    w.y=(p2.y-p0.y);
    w.z=(p2.z-p0.z);
    prodvec[0]=(v.y * w.z - v.z * w.y);
    prodvec[1]=(v.z * w.x - v.x * w.z);
    prodvec[2]=(v.x * w.y - v.y * w.x);

    //cout<<prodvec[0]<<endl<<prodvec[1]<<endl<<prodvec[2]<<endl;*/

    area=0.5*sqrt(pow(tot[0],2) + pow(tot[1],2) + pow(tot[2],2));
    //cout<<area<<endl;
    return area;
}
