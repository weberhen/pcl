/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2015-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <iostream>

#include <boost/format.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/io_exception.h>
#include <pcl/io/freenect2_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl::console;

void
printHelp (int,
           char **argv)
{
  PCL_INFO ("Creates a point cloud grabber for the kinect2\n");
  PCL_INFO ("Accepts as constructor parameter a string specifying a kinect2 serial number\n");
}

template <typename PointT>
class Freenect2Viewer
{

  public:

    typedef pcl::PointCloud<PointT> PointCloudT;

    Freenect2Viewer (pcl::Freenect2Grabber& grabber) :
        grabber_ (grabber),
        viewer_ ("Freenect2 Viewer")
    {
      viewer_.setCameraFieldOfView (0.785398);  // approximately 45 degrees
      viewer_.setCameraPosition (0, 0, 0, 0, 0, 1, 0, 1, 0);
      viewer_.registerKeyboardCallback (&Freenect2Viewer::keyboardCallback, *this);
      viewer_.registerPointPickingCallback (&Freenect2Viewer::pointPickingCallback, *this);
    }

    ~Freenect2Viewer ()
    {
      connection_.disconnect ();
    }

    void
    run ()
    {
      boost::function<void
      (const typename PointCloudT::ConstPtr&)> f = boost::bind (&Freenect2Viewer::cloudCallback, this, _1);
      connection_ = grabber_.registerCallback (f);
      grabber_.start ();

      while (!viewer_.wasStopped ())
      {
        if (new_cloud_)
        {
          boost::mutex::scoped_lock lock (new_cloud_mutex_);
          if (!viewer_.updatePointCloud (new_cloud_, "cloud"))
          {
            viewer_.addPointCloud (new_cloud_, "cloud");
          }

          last_cloud_ = new_cloud_;
          new_cloud_.reset ();
        }
        viewer_.spinOnce ();
      }
      grabber_.stop ();
    }

  private:

    void
    cloudCallback (typename PointCloudT::ConstPtr cloud)
    {
      if (!viewer_.wasStopped ())
      {
        boost::mutex::scoped_lock lock (new_cloud_mutex_);
        new_cloud_ = cloud;
      }
    }

    void
    keyboardCallback (const pcl::visualization::KeyboardEvent& event,
                      void*)
    {
      if (event.keyDown ())
      {
        if (event.getKeyCode () == 's')
        {
          boost::format fmt ("RS_%s_%u.pcd");
          std::string fn = boost::str (fmt % grabber_.getDeviceSerialNumber ().c_str () % last_cloud_->header.stamp);
          pcl::io::savePCDFileBinaryCompressed (fn, *last_cloud_);
          pcl::console::print_info ("Saved point cloud: ");
          pcl::console::print_value (fn.c_str ());
          pcl::console::print_info ("\n");
        }
      }
    }

    void
    pointPickingCallback (const pcl::visualization::PointPickingEvent& event,
                          void*)
    {
      float x, y, z;
      event.getPoint (x, y, z);
      pcl::console::print_info ("Picked point at ");
      pcl::console::print_value ("%.3f", x);
      pcl::console::print_info (", ");
      pcl::console::print_value ("%.3f", y);
      pcl::console::print_info (", ");
      pcl::console::print_value ("%.3f\n", z);
    }

    pcl::Freenect2Grabber& grabber_;
    pcl::visualization::PCLVisualizer viewer_;
    boost::signals2::connection connection_;

    mutable boost::mutex new_cloud_mutex_;
    typename PointCloudT::ConstPtr new_cloud_;
    typename PointCloudT::ConstPtr last_cloud_;
};

int
main (int argc,
      char** argv)
{
  print_info ("Viewer for Freenect2 devices (run with --help for more information)\n", argv[0]);

  if (find_switch (argc, argv, "--help") || find_switch (argc, argv, "-h"))
  {
    printHelp (argc, argv);
    return (0);
  }

  std::string device_id;

  if (argc == 1)  // single argument
  {
    device_id = "";
    print_info ("Creating a grabber for the first available device\n");
  }
  else
  {
    device_id = argv[argc - 1];
    print_info ("Creating a grabber for device ");
    print_value ("%s\n", device_id.c_str ());
  }

  try
  {
    pcl::Freenect2Grabber grabber (pcl::OPENGL);

    Freenect2Viewer<pcl::PointXYZRGB> viewer (grabber);
    viewer.run ();
  }
  catch (pcl::io::IOException& e)
  {
    print_error ("Failed to create a grabber: %s\n", e.what ());
    return (1);
  }
  return (0);
}
