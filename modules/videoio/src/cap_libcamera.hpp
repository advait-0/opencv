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
public:
   CvCapture_libcamera_proxy(int index) 
    {
        cm_ = std::make_unique<CameraManager>();
        cm_->start();
        cameraId_ = cm_->cameras()[index]->id();
        cam_init();
    }
   CvCapture_libcamera_proxy()
    {
        cm_ = std::make_unique<CameraManager>();
        cm_->start();
        cameraId_ = cm_->cameras()[0]->id();
        cam_init();
    }
    bool isOpened() const CV_OVERRIDE
    {
       return opened_;
    }
    bool open(int index);
    bool grabFrame() CV_OVERRIDE;
    bool retrieveFrame(int, OutputArray) CV_OVERRIDE;
    virtual double getProperty(int) const CV_OVERRIDE;
    virtual bool setProperty(int, double) CV_OVERRIDE;
    int mapFrameBuffer(const FrameBuffer *buffer);
    int convertToRgb(libcamera::Request *req, OutputArray &outImage);
    virtual int getCaptureDomain() CV_OVERRIDE { return CAP_LIBCAMERA; }

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
    static void requestComplete(Request *request);
    static std::queue<Request*> completedRequests_;
    bool handled;
    StreamConfiguration streamConfig_;
    std::unique_ptr<CameraConfiguration> config_;
    std::unique_ptr<CameraManager> cm_;
    std::shared_ptr<Camera> camera_;
    std::string cameraId_;
    std::vector<libcamera::Span<uint8_t>> planes_;
    std::vector<libcamera::Span<uint8_t>> maps_;
    std::vector<std::unique_ptr<Request>> requests_;
    std::unique_ptr<FrameBufferAllocator> allocator_;
    int frameWidth;
    int frameHeight;
    unsigned int allocated_;
    bool opened_=false;
    

    protected:
    void cam_init();
    void cam_init(int index);
};
}
 
#endif
