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

pcl::k4w2Grabber::k4w2Grabber(processor p)
    : Grabber(),
    is_running_(false),
    qnan_(std::numeric_limits<float>::quiet_NaN()),
    undistorted_(512, 424, 4),
    registered_(512, 424, 4),
    big_mat_(1920, 1082, 4)
{

    switch(p){
        case CPU:
            std::cout << "creating CPU processor" << std::endl;
            dev_ = freenect2_.openDefaultDevice(new libfreenect2::CpuPacketPipeline());
            std::cout << "created" << std::endl;
            break;
        case OPENCL:
            std::cout << "creating OpenCL processor" << std::endl;
            dev_ = freenect2_.openDefaultDevice(new libfreenect2::OpenCLPacketPipeline());
            break;
        case OPENGL:
            std::cout << "creating OpenGL processor" << std::endl;
            dev_ = freenect2_.openDefaultDevice(new libfreenect2::OpenGLPacketPipeline());
            break;
        default:
            std::cout << "creating CPU processor" << std::endl;
            dev_ = freenect2_.openDefaultDevice(new libfreenect2::CpuPacketPipeline());
            break;
    }
    

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

void 
pcl::k4w2Grabber::prepareMake3D(const libfreenect2::Freenect2Device::IrCameraParams & depth_p)
{
    const int w = 512;
    const int h = 424;
    float * pm1 = colmap_.data();
    float * pm2 = rowmap_.data();
    for(int i = 0; i < w; ++i)
    {
        *pm1++ = (i - depth_p.cx + 0.5) / depth_p.fx;
    }
    for (int i = 0; i < h; i++)
    {
        *pm2++ = (i - depth_p.cy + 0.5) / depth_p.fy;
    }
}

void pcl::k4w2Grabber::run()
{
    dev_->start();

    prepareMake3D(dev_->getIrCameraParams());

    registration_ = new libfreenect2::Registration(dev_->getIrCameraParams(), dev_->getColorCameraParams());

    while (is_running_)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzrgb_cloud;

        if (need_xyzrgb_)
        {
            listener_->waitForNewFrame(frames_);
            libfreenect2::Frame * rgb = frames_[libfreenect2::Frame::Color];
            libfreenect2::Frame * depth = frames_[libfreenect2::Frame::Depth];

            registration_->apply(rgb, depth, &undistorted_, &registered_, true, &big_mat_, map_);
            const short w = undistorted_.width;
            const short h = undistorted_.height;
            bool is_dense = true;

            xyzrgb_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>(w, h));

            const float * itD0 = (float *)undistorted_.data;
            const char * itRGB0 = (char *)registered_.data;
            pcl::PointXYZRGB * itP = &xyzrgb_cloud->points[0];
            
            for(int y = 0; y < h; ++y){

                const unsigned int offset = y * w;
                const float * itD = itD0 + offset;
                const char * itRGB = itRGB0 + offset * 4;
                const float dy = rowmap_(y);

                for(size_t x = 0; x < w; ++x, ++itP, ++itD, itRGB += 4)
                {
                    const float depth_value = *itD / 1000.0f;
                    
                    if(!std::isnan(depth_value) && !(std::abs(depth_value) < 0.0001)){
        
                        const float rx = colmap_(x) * depth_value;
                        const float ry = dy * depth_value;               
                        itP->z = depth_value;
                        itP->x = rx;
                        itP->y = ry;

                        itP->b = itRGB[0];
                        itP->g = itRGB[1];
                        itP->r = itRGB[2];
                    } else {
                        itP->z = qnan_;
                        itP->x = qnan_;
                        itP->y = qnan_;

                        itP->b = qnan_;
                        itP->g = qnan_;
                        itP->r = qnan_;
                        is_dense = false;
                    }
                }
            }
            xyzrgb_cloud->is_dense = is_dense;
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
