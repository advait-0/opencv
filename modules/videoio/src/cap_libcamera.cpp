#include "precomp.hpp"
#include <iostream>
#include <memory>
#include <opencv2/core.hpp>
#include <libcamera/libcamera.h>

using namespace cv;
using namespace libcamera;

struct CvCapture_libcamera
{
    bool open(const char* filename, const VideoCaptureParameters& params);
    void close();
    double getProperty(int) const;
    bool grabFrame();
};

namespace cv {
namespace {
class CvCapture_libcamera_proxy CV_FINAL : public cv::IVideoCapture
{
    public:
    bool isOpened_ = false;
    bool grabFrame_=false;
    bool retrieveFrame_=false;
    CvCapture_libcamera_proxy() 
    {
        libcameraCapture=NULL;
       
        

        // Check if cameras are available

        grabFrame_=false;
        retrieveFrame_=false;
        
    }
    //Return necessary values to resolve build errors
    bool isOpened() const CV_OVERRIDE
    {
        std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
        cm->start();
        if (cm->cameras().empty()) 
        {
            
            std::cout << "No cameras available" << std::endl;
            cm->stop();
            isOpened_ = false;
        }
        else 
        {
            isOpened_ = true;
            std::cout << "Cameras available" << std::endl;
        }
      return isOpened_;
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

    protected:
    CvCapture_libcamera* libcameraCapture;
};
} //anonymous namespace
    cv::Ptr<cv::IVideoCapture> create_libcamera_capture_cam(int index)
{
    cv::Ptr<CvCapture_libcamera_proxy> capture = cv::makePtr<CvCapture_libcamera_proxy>();
    std::cout<<index;
     // Testing
    if (capture && capture->isOpened())
        return capture;
    return cv::Ptr<cv::IVideoCapture>();
} 
}//cv namespace

int main()
{
    std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
    cm->start();
    if (cm->cameras().empty()) 
    {
        std::cout << "No cameras were identified on the system" << std::endl;
        cm->stop();
        return EXIT_FAILURE;
    }

    for (const auto& cam : cm->cameras())
        std::cout << "Camera Found : " << cam->id() << std::endl;

    
    CvCapture_libcamera_proxy *cap = nullptr;
    int ind=0;
    cap = new CvCapture_libcamera_proxy();
        std::cout<<ind;
        if (cap->isOpened())
        {
            //Call the create_libcamera_capture_cam function and pass int parameter
            cv::Ptr<cv::IVideoCapture> videoCapture = create_libcamera_capture_cam(ind);
            if (videoCapture)
            {
            std::cout << "Camera available main";
            return true;
            }
            std::cout<<"Camera available main";
            return true;
        }

    
    delete cap;
    return 0;
}