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

#include <pcl/io/k4w2_grabber.h>
#include <libfreenect2/packet_pipeline.h>

pcl::k4w2Grabber::k4w2Grabber()
    : Grabber(),
    is_running_(false)
{
    dev_ = freenect2_.openDefaultDevice(new libfreenect2::OpenCLPacketPipeline());
    
    listener_ = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
    
    if (dev_ == 0) {
        PCL_ERROR("no device connected or failure opening the default one!\n");
        exit(1);
    }
    
    dev_->setColorFrameListener(listener_);
    dev_->setIrAndDepthFrameListener(listener_);

    point_cloud_rgb_signal_ = createSignal<sig_cb_k4w2_point_cloud_rgb>();
}

pcl::k4w2Grabber::~k4w2Grabber() throw ()
{
    dev_->stop();
    dev_->close();

    disconnect_all_slots<sig_cb_k4w2_point_cloud_rgb>();
}

void pcl::k4w2Grabber::start()
{
    if (!is_running_)
    {
        need_xyzrgb_ = num_slots<sig_cb_k4w2_point_cloud_rgb>() > 0;

        frequency_.reset();

        is_running_ = true;
        thread_ = boost::thread(&k4w2Grabber::run, this);
    }
}

void pcl::k4w2Grabber::stop()
{
    if (is_running_)
    {
        is_running_ = false;
        thread_.join();
    }
}

bool pcl::k4w2Grabber::isRunning() const
{
    return (is_running_);
}

float
pcl::k4w2Grabber::getFramesPerSecond() const
{
    boost::mutex::scoped_lock lock(fps_mutex_);
    return (frequency_.getFrequency());
}

void pcl::k4w2Grabber::run()
{
    dev_->start();

    registration_ = new libfreenect2::Registration(dev_->getIrCameraParams(), dev_->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

    while (is_running_)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzrgb_cloud;

        if (need_xyzrgb_)
        {
            listener_->waitForNewFrame(frames_);
            libfreenect2::Frame *rgb = frames_[libfreenect2::Frame::Color];
            libfreenect2::Frame *ir = frames_[libfreenect2::Frame::Ir];
            libfreenect2::Frame *depth = frames_[libfreenect2::Frame::Depth];

            registration_->apply(rgb, depth, &undistorted, &registered);

            xyzrgb_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>(registered.width,registered.height));
            xyzrgb_cloud->is_dense = false;

            for (size_t i = 0; i < registered.height; i++)
            {
                for (size_t j = 0; j < registered.width; j++)
                {
                    float x;
                    float y;
                    float z;
                    float frgb;
                    registration_->getPointXYZRGB(&undistorted, &registered, i, j, x, y, z, frgb);

                    int pt = i * registered.width + j;
                    
                    xyzrgb_cloud->points[pt].x = x;
                    xyzrgb_cloud->points[pt].y = y;
                    xyzrgb_cloud->points[pt].z = z;
                    xyzrgb_cloud->points[pt].rgb = *reinterpret_cast<float*>(&frgb);
                }
            }
        }

        fps_mutex_.lock();
        frequency_.event();
        fps_mutex_.unlock();

        if (need_xyzrgb_)
            point_cloud_rgb_signal_->operator () (xyzrgb_cloud);
        
        listener_->release(frames_);
    }

    delete registration_;
}