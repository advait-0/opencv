#include <iostream>
#include <memory>
#include "precomp.hpp"

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <libcamera/libcamera.h>

#define TIMEOUT_SEC 3

using namespace cv;
using namespace libcamera;

namespace {

class CvCapture_libcamera_proxy CV_FINAL : public cv::IVideoCapture
{
    bool isOpened_ = false;

public:
    CvCapture_libcamera_proxy()
    {
        isOpened_ = false;
    }

    virtual bool isOpened() CV_OVERRIDE
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

        return isOpened_;
    }
};

cv::Ptr<cv::IVideoCapture> create_libcamera_capture_cam()
{
    cv::Ptr<CvCapture_libcamera_proxy> capture = cv::makePtr<CvCapture_libcamera_proxy>();
    if (capture && capture->isOpened())
        return capture;
    return cv::Ptr<cv::IVideoCapture>();
}

} // namespace

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

    return 0;
}
