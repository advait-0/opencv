// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.
// Copyright (C) 2023, Advait Dhamorikar all rights reserved.

#ifdef HAVE_LIBCAMERA

#include <iostream>
#include <sys/mman.h>
#include <errno.h>
#include <memory>
#include <queue>
#include <map>
#include <unistd.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <libcamera/libcamera.h>
#include <libcamera/framebuffer.h>
#include <libcamera/base/span.h>

using namespace cv;
using namespace libcamera;

namespace cv{
class CvCapture_libcamera_proxy CV_FINAL : public cv::IVideoCapture
{
int width_set = 0;
int height_set = 0;
public:
    CvCapture_libcamera_proxy(int index = 0) 
    {
        cm_ = std::make_unique<CameraManager>();
        cm_->start();
        cameraId_ = cm_->cameras()[index]->id();
        cam_init(index);
    }
    bool isOpened() const CV_OVERRIDE { return opened_; }
    bool icvSetFrameSize(int, int);
    bool open();
    bool grabFrame() CV_OVERRIDE;
    bool retrieveFrame(int, OutputArray) CV_OVERRIDE;
    virtual double getProperty(int) const CV_OVERRIDE;
    virtual bool setProperty(int, double) CV_OVERRIDE;
    int mapFrameBuffer(const FrameBuffer *buffer);
    int convertToRgb(libcamera::Request *req, OutputArray &outImage);
    virtual int getCaptureDomain() CV_OVERRIDE { return cv::CAP_LIBCAMERA; }
   

    ~CvCapture_libcamera_proxy()
    {
        if (opened_)
        {
            camera_->stop();
            allocator_.reset();
            camera_->release();
            cm_->stop();
        }
    }
    
    private:
    bool getLibcameraPixelFormat(int value) 
    {
    switch (value) 
       {
        case 0:
            pixelFormat_ = libcamera::formats::MJPEG;
            return true;
        case 1:
            pixelFormat_ = libcamera::formats::YUYV;
            return true;
        default:
            pixelFormat_ = libcamera::formats::MJPEG;
            return true; // Default value
       }
    }
    bool getCameraConfiguration(int value)
    {
     switch (value) 
       {
        case 0:
            strcfg_ = StreamRole::Raw;
            return true;
        case 1:
            strcfg_ = StreamRole::StillCapture;
            return true;
        case 2:
            strcfg_ = StreamRole::VideoRecording;
            return true;
        case 3:
            strcfg_ = StreamRole::Viewfinder;
            return true;
        default:
            strcfg_ = StreamRole::VideoRecording;
            return true;// Default value
        }
    }
    static void requestComplete(Request *request);
    static std::queue<Request*> completedRequests_;
    bool handled;
    StreamConfiguration streamConfig_;
    StreamRole strcfg_;
    PixelFormat pixelFormat_;
    std::unique_ptr<CameraConfiguration> config_;
    std::unique_ptr<CameraManager> cm_;
    std::shared_ptr<Camera> camera_;
    std::string cameraId_;
    std::vector<libcamera::Span<uint8_t>> planes_;
    std::vector<libcamera::Span<uint8_t>> maps_;
    std::vector<std::unique_ptr<Request>> requests_;
    std::unique_ptr<FrameBufferAllocator> allocator_;
    int width_, height_;
    int pixFmt_;
    int propFmt_;
    unsigned int allocated_;
    bool opened_ = false;
    
    protected:
    void cam_init();
    void cam_init(int index);
};
}
 
#endif
