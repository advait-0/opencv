#include <iomanip>
#include <iostream>
#include <memory>
#include "precomp.hpp"

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <libcamera/libcamera.h>

#define TIMEOUT_SEC 3

using namespace cv;
using namespace libcamera;


cv::Ptr<cv::IVideoCapture> create_libcamera_capture_cam(int index);

class CvCapture_libcamera_proxy CV_FINAL : public cv::IVideoCapture
{
    bool isOpened_ = false;
   
    public:
    CvCapture_libcamera_proxy()
    {
     std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
     cm->start();
        

    if(cm->cameras().empty())
    {
        isOpened_= false;
        std::cout << "Cameras not available" << std::endl;
    }
    else
    {
        isOpened_= true;
        std::cout << "Cameras available" << std::endl;
    }
    
    }

    //  ~CvCapture_libcamera_proxy()
    // {
    //     if (isOpened_){
    //         cm->stop();
    //     }
    // }

    bool isOpened() const CV_OVERRIDE
    {
        return isOpened_;
    }

};
// bool isOpened() const { return capture  != nullptr; }
// cv::Ptr<cv::IVideoCapture> create_libcamera_capture_cam(int index)
// {
//     Ptr<IVideoCapture> capture = makePtr<CameraManager>(index);
//     // std::unique_ptr<CameraManager> cm = 
   



//     // CvCapture_libcamera* capture = new CvCapture_libcamera();

//     // if (capture->open(index))
//     //     return cv::makePtr<LegacyCapture>(capture);

//     // delete capture;
//     // return nullptr;
// }




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
	std::cout << "Camera Found : " 
		  << cam->id() << std::endl;

    return 0;
}



