#include "precomp.hpp"

#include <iostream>
#include <memory>
#include "cap_interface.hpp"
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <libcamera/libcamera.h>

using namespace cv;
using namespace libcamera;

namespace cv{
class CvCapture_libcamera_proxy CV_FINAL : public cv::IVideoCapture
{
public:
    CvCapture_libcamera_proxy()
    {
        std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
        cm->start();
        static std::shared_ptr<Camera> camera;
    }

    CvCapture_libcamera_proxy(int index)
    {
        std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
        cm->start();
        static std::shared_ptr<Camera> camera;
        std::string cameraId = cm->cameras()[index]->id();
	    if(!cameraId.empty())
        {
          open(index);
        }
    }

    bool open(int index);

    bool isOpened() const CV_OVERRIDE
    {
       return opened;
    }

    bool grabFrame() CV_OVERRIDE;


    bool retrieveFrame(int, OutputArray) CV_OVERRIDE
    {
        return false;
    }

    virtual int getCaptureDomain() CV_OVERRIDE { return CAP_LIBCAMERA; }

    private:
    bool opened=false;
    bool grabFrame_check;
    bool grabbedinOpen;

    protected:
    void cam_init();

};

void CvCapture_libcamera_proxy::cam_init()
{
     std::cout<<"cam_init test";
}

bool CvCapture_libcamera_proxy::open(int index)
{
    std::cerr<<"Entered Open";
    try
    {
        std::string cameraId = cm->cameras()[index]->id();
        camera=cm->get(cameraId);
        camera->acquire();

        std::unique_ptr<CameraConfiguration> config =
		camera->generateConfiguration( { StreamRole::VideoRecording } );

        config->validate();
	    std::cout << "Validated viewfinder configuration is: "
		  << streamConfig.toString() << std::endl;
	    camera->configure(config.get());

        opened = true;
        // return true;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        std::cout<<"Try failed";
        opened=false;
    }
    return opened;
}

bool CvCapture_libcamera_proxy::grabFrame()
{
    FrameBufferAllocator *allocator = new FrameBufferAllocator(camera);

	for (StreamConfiguration &cfg : *config) {
		int ret = allocator->allocate(cfg.stream());
		if (ret < 0) {
			std::cerr << "Can't allocate buffers" << std::endl;
			return false;
		}

		size_t allocated = allocator->buffers(cfg.stream()).size();
		std::cout << "Allocated " << allocated << " buffers for stream" << std::endl;
	}

    	Stream *stream = streamConfig.stream();
	const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator->buffers(stream);
	std::vector<std::unique_ptr<Request>> requests;
	for (unsigned int i = 0; i < buffers.size(); ++i) {
		std::unique_ptr<Request> request = camera->createRequest();
		if (!request)
		{
			std::cerr << "Can't create request" << std::endl;
			return false;
		}

		const std::unique_ptr<FrameBuffer> &buffer = buffers[i];
		int ret = request->addBuffer(stream, buffer.get());
		if (ret < 0)
		{
			std::cerr << "Can't set buffer for request"
				  << std::endl;
			return false;
		}

		ControlList &controls = request->controls();
		controls.set(controls::Brightness, 0.5);

		requests.push_back(std::move(request));
	}

    // camera->requestCompleted.connect(requestComplete);

    camera->start();
	for (std::unique_ptr<Request> &request : requests)
    {
		camera->queueRequest(request.get());
    }
    std::cout<<"Buffers have been queued";
    grabFrame_check=true;
    return true;
}

cv::Ptr<cv::IVideoCapture> create_libcamera_capture_cam(int index)
{
    cv::Ptr<CvCapture_libcamera_proxy> capture = cv::makePtr<CvCapture_libcamera_proxy>();

    std::cout << "index passed to create_libcamera_capture_cam : " << index << std::endl;
    if (capture)
        return capture;

    return cv::Ptr<cv::IVideoCapture>();
}

}//namespace cv 
