/*
 * Copyright (C) 2018
 * Authors: Nicolò Genesio
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <MobilityMapModule.h>
#include <pcl/common/time.h>

using namespace yarp::os;



MobilityMapModule::MobilityMapModule(int _nClouds): configured(false), nClouds(_nClouds),
                                                    offline(true), cloud_start(0),
                                                    first(true), imageWidth(0),
                                                    imageHeight(0),focalLengthX(1.0),
                                                    focalLengthY(1.0), openniGrabber(nullptr),
                                                    period(0.0)
{
}


MobilityMapModule::MobilityMapModule(): configured(false), nClouds(0),
                                        offline(true), cloud_start(0),
                                        first(true), imageWidth(0),
                                        imageHeight(0),focalLengthX(1.0),
                                        focalLengthY(1.0), openniGrabber(nullptr),
                                        period(0.0)
{
}


void MobilityMapModule::cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
{
  static unsigned count = 0;
  static double last = pcl::getTime ();
  if (++count == 30)
  {
    double now = pcl::getTime ();
    std::cout << "distance of center pixel :" << cloud->points [(cloud->width >> 1) * (cloud->height + 1)].z << " mm. Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
    count = 0;
    last = now;
  }
}

bool MobilityMapModule::configure(yarp::os::ResourceFinder &rf)
{
    if (rf.check("period"))
    {
        period = rf.find("period").asDouble();
    }
    if (rf.find("offline").asBool())
    {
        offline = true;
    }
    else
    {
        offline = false;
    }

    if (offline)
    {
        if (rf.check("firstCloud"))
        {
            cloud_start = rf.find("firstCloud").asInt();
        }
    }
    else
    {
      // create a new grabber for OpenNI devices
      openniGrabber = new pcl::OpenNIGrabber();

      // make callback function from member function
      boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
        boost::bind (&MobilityMapModule::cloud_cb_, this, _1);

      // connect callback function for desired signal. In this case its a point cloud with color values
      boost::signals2::connection c = openniGrabber->registerCallback (f);

      // start receiving point clouds
      openniGrabber->start();

      yarp::os::Time::delay(0.5);

      if (!openniGrabber->isRunning())
      {
          yError()<<"Error opening the device...";
          return false;
      }

    }
    return true;

}

bool MobilityMapModule::updateModule()
{
    return true;
}

double MobilityMapModule::getPeriod()
{
    return period;
}


bool MobilityMapModule::interruptModule()
{
    return close();
}
bool MobilityMapModule::close()
{
    // stop the grabber
    openniGrabber->stop();
    return true;
}
