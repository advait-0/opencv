#include <iostream>
#include <memory>
#include "precomp.hpp"
#include "cap_interface.hpp"
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <libcamera/libcamera.h>

#define TIMEOUT_SEC 3

using namespace cv;
using namespace libcamera;

namespace cv{
class CvCapture_libcamera_proxy CV_FINAL : public cv::IVideoCapture
{
public:
    CvCapture_libcamera_proxy()
    {
    }

    bool open(int index);
    bool isOpened() const CV_OVERRIDE
    {
       return opened;
    }

    bool grabFrame() CV_OVERRIDE
    {
        return false;
    }

    bool retrieveFrame(int, OutputArray) CV_OVERRIDE
    {
        return false;
    }

    virtual int getCaptureDomain() CV_OVERRIDE { return CAP_LIBCAMERA; }

    private:
    bool opened;
};

bool CvCapture_libcamera_proxy::open(int index)
{
    if (isOpened()) 
    {
        std::cout<<"Line 61 is opened works";
    }
    try
    {
        std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();

        cm->start();
        if (cm->cameras().empty()) 
        {
            std::cout << "No cameras available" << std::endl;
        } 
        else 
        {
            std::cout << "Cameras available: " << cm->cameras().size() << std::endl;
            throw 407;
        }
	    cm->stop();
        opened = true;
        return true;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        std::cout<<"Try failed";
        return false;
    }
    
}

cv::Ptr<cv::IVideoCapture> create_libcamera_capture_cam(int index)
{
    cv::Ptr<CvCapture_libcamera_proxy> capture = cv::makePtr<CvCapture_libcamera_proxy>();

    std::cout << "index passed to create_libcamera_capture_cam : " << index << std::endl;
    if (capture && capture->isOpened())
        return capture;

    return cv::Ptr<cv::IVideoCapture>();
} 
}//namespace cv 
