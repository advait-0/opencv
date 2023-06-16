#include <iomanip>
#include <iostream>
#include <memory>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <libcamera/libcamera.h>

#define TIMEOUT_SEC 3

using namespace cv;
using namespace libcamera;
<<<<<<< HEAD



cv::Ptr<cv::IVideoCapture> create_libcamera_capture_cam(int index)
{
    std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
    cm->start();
    if(cm->cameras().empty!=false)
    {

    }
    Ptr<IVideoCapture> capture = makePtr<DigitalCameraCapture>(index);


    // CvCapture_libcamera* capture = new CvCapture_libcamera();

    // if (capture->open(index))
    //     return cv::makePtr<LegacyCapture>(capture);

    // delete capture;
    // return nullptr;
}
=======
>>>>>>> 60f9d2159746b43c1a14ee6270c6adce34473720

cv::Ptr<cv::IVideoCapture> create_libcamera_capture_file(const std::string& filename)
{
    CvCapture_libcamera* capture = new CvCapture_libcamera();

    if (capture->open(filename.c_str()))
        return cv::makePtr<LegacyCapture>(capture);

    delete capture;
    return nullptr;
}
}


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



