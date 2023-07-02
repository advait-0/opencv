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

// namespace {


class CvCapture_libcamera_proxy CV_FINAL : public cv::IVideoCapture
{
    

public:
    bool isOpened_ = false;
    bool grabFrame_=false;
    bool retrieveFrame_=false;
    CvCapture_libcamera_proxy()
    {
        std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
        cm->start();

        // Check if cameras are available
        if (cm->cameras().empty()) {
            std::cout << "No cameras available" << std::endl;
            cm->stop();
            isOpened_ = false;
        }
        else {
            isOpened_ = true;
            std::cout << "Cameras available" << std::endl;
        }
        grabFrame_=false;
        retrieveFrame_=false;
        
    }

    //Return necessary values to resolve build errors

    bool isOpened() const CV_OVERRIDE
    {
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
};

//Just a dummy function for now, to check for build errors
cv::Ptr<cv::IVideoCapture> create_libcamera_capture_cam(int index)
{
    cv::Ptr<CvCapture_libcamera_proxy> capture = cv::makePtr<CvCapture_libcamera_proxy>();
    std::cout<<index;
     // Testing
    if (capture && capture->isOpened())
        return capture;
    return cv::Ptr<cv::IVideoCapture>();
   
} 
// }// namespace



int main()
{
    std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
    cm->start();

    if (cm->cameras().empty()) {
        std::cout << "No cameras were identified on the system" << std::endl;
        cm->stop();
        return EXIT_FAILURE;
    }

    for (const auto& cam : cm->cameras())
        std::cout << "Camera Found : " << cam->id() << std::endl;

    
    CvCapture_libcamera_proxy *cap = 0;
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
