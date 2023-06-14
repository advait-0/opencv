#include <iomanip>
#include <iostream>
#include <memory>

#include <libcamera/libcamera.h>
#include <opencv2/core/utils/logger.hpp>
#include <opencv2/core/utils/filesystem.hpp>


#define TIMEOUT_SEC 3

using namespace libcamera;
static std::shared_ptr<Camera> camera;

int main()
{
	
	
	std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
	cm->start();

	
	for (auto const &camera : cm->cameras())
		std::cout << " - " << cameraName(camera.get()) << std::endl;


	if (cm->cameras().empty()) {
		std::cout << "No cameras were identified on the system"<< std::endl;
		cm->stop();
		return EXIT_FAILURE;
	}

	std::string cameraId = cm->cameras()[0]->id();
	camera = cm->get(cameraId);
	std::cout<<"Cameras available yayy";
	return cv::CAP_LIBCAMERA;

}
