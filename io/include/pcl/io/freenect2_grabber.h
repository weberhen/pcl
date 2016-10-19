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

#ifndef PCL_IO_FREENECT2_GRABBER_H
#define PCL_IO_FREENECT2_GRABBER_H

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>

#include "/gel/usr/heweb4/freenect2/include/libfreenect2/libfreenect2.hpp"
//#include <libfreenect2/libfreenect2.hpp>
#include "/gel/usr/heweb4/freenect2/include/libfreenect2/frame_listener_impl.h"
//#include <libfreenect2/frame_listener_impl.h>
#include "/gel/usr/heweb4/freenect2/include/libfreenect2/registration.h"
//#include <libfreenect2/registration.h>
#include "/gel/usr/heweb4/freenect2/include/libfreenect2/packet_pipeline.h"
//#include <libfreenect2/packet_pipeline.h>
#include <Eigen/Core>

#include <limits>

namespace pcl
{
  enum processor
  {
    CPU, OPENCL, OPENGL, CUDA
  };

class PCL_EXPORTS Freenect2Grabber : public Grabber
{
  public:
  typedef
  void (sig_cb_freenect2_point_cloud_rgb)
  (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&);

  Freenect2Grabber(processor p = CPU, std::string serial = std::string());
  virtual
  ~Freenect2Grabber() throw();

  virtual bool
  isRunning() const;

  virtual void
  start();

  virtual void
  stop();

  virtual std::string
  getName() const
  {
    return (std::string("Freenect2Grabber"));
  }

  virtual std::string
  getDeviceSerialNumber() const
  {
    return dev_->getSerialNumber();
  }

  virtual float
  getFramesPerSecond() const;

  private:
  void
  run();

  void prepareMake3D(const libfreenect2::Freenect2Device::IrCameraParams & depth_p);

  // Signals to indicate whether new clouds are available
  boost::signals2::signal<sig_cb_freenect2_point_cloud_rgb>* point_cloud_rgb_signal_;

  boost::thread thread_;

  bool is_running_;

  /// Indicates whether there are subscribers for PointXYZRGBA signal,
  /// computed and stored on start()
  bool need_xyzrgb_;

  EventFrequency frequency_;
  mutable boost::mutex fps_mutex_;

  libfreenect2::Registration* registration_;

  libfreenect2::Freenect2 freenect2_;
  libfreenect2::Freenect2Device * dev_;
  libfreenect2::SyncMultiFrameListener * listener_;
  libfreenect2::PacketPipeline * pipeline_;
  libfreenect2::FrameMap frames_;
  libfreenect2::Frame undistorted_, registered_, big_mat_;
  Eigen::Matrix<float,512,1> colmap_;
  Eigen::Matrix<float,424,1> rowmap_;
  std::string serial_;
  int map_[512 * 424];
  float qnan_;
};
}

#endif // PCL_IO_FREENECT2_GRABBER_H
