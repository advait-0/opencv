#include <iomanip>
#include <iostream>
#include <memory>

#include <libcamera/libcamera.h>

#define TIMEOUT_SEC 3

using namespace libcamera;

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


Ptr<IVideoCapture> create_libcamera_capture_cam(int index)
{
    cv::CvCapture_libcamera* capture = new cv::CvCapture_libcamera();

    if (capture->open(index))
        return makePtr<LegacyCapture>(capture);

    delete capture;
    return NULL;
}

Ptr<IVideoCapture> create_libcamera_capture_file(const std::string &filename)
{
    cv::CvCapture_libcamera* capture = new cv::CvCapture_libcamera();

    if (capture->open(filename.c_str()))
        return makePtr<LegacyCapture>(capture);

    delete capture;
    return NULL;
}

