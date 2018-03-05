/*
 * Copyright (C) 2018
 * Authors: Nicolò Genesio
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef MOBILITYMAPMODULE_H
#define MOBILITYMAPMODULE_H

#include "yarp/os/all.h"
#include "yarp/sig/all.h"
#include "opencv2/core.hpp"
#include <pcl/io/openni_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class MobilityMapModule : public yarp::os::RFModule
{
    bool configured, offline;
    int nClouds;

    int cloud_start;
    bool first;
    size_t imageWidth, imageHeight;
    float focalLengthX, focalLengthY;
    pcl::Grabber* openniGrabber;
    double period;
    //pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloudPtr; // do I need it??
//    ThreadFeature* threadFeatureR; //thread that reads from a buffer of R images and writes to a buffer of R features
//    ThreadFeature* threadFeatureL; //thread that reads from a buffer of L images and writes to a buffer of L features
//    ThreadDescriptor* threadDescriptorR; //thread that reads from a buffer of R features and writes to a prioritybuffer(*) of descriptors
//    ThreadDescriptor* threadDescriptorL; //thread that reads from a buffer of L features and writes to a prioritybuffer(*) of descriptors
//    ThreadMatching* threadMatching; //thread that reads from a prioritybuffer of descriptors, does matching and compute R&t

//    yarp::os::AtomicBuffer<SlamType> bufferPointCloud;
//    yarp::os::AtomicBuffer<SlamType> bufferFeatureR;  // TODO One day will implement like this...
//    yarp::os::AtomicBuffer<SlamType> bufferFeatureL;
//    yarp::os::AtomicBuffer<SlamType> bufferDescriptorR;
//    yarp::os::AtomicBuffer<SlamType> bufferDescriptorL;
//    yarp::os::AtomicBuffer<SlamType> bufferMatching;


public:
    MobilityMapModule();
    MobilityMapModule(int _nClouds);
    bool configure(yarp::os::ResourceFinder &rf);
    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
    bool updateModule();
    double getPeriod();
    bool interruptModule();
    bool close();
    bool interrupted;
};

#endif // MOBILITYMAPMODULE_H
