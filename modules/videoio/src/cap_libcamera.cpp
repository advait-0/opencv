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
   CvCapture_libcamera_proxy(int index = 0)
    {
        ind = index;
        cm = std::make_unique<CameraManager>();
        cm->start();
        cameraId = cm->cameras()[ind]->id();
        cam_init();
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
    int ind;
    StreamConfiguration streamConfig;
    static std::unique_ptr<CameraConfiguration> config;
    static std::unique_ptr<CameraManager> cm;
    static std::shared_ptr<Camera> camera;
    static std::string cameraId;
    bool opened=false;
    bool grabFrame_check;
    bool grabbedinOpen;

    protected:
    void cam_init();
    void cam_init(int index);
    static void processRequest(Request *request);
    static void requestComplete(Request *request);

};

std::unique_ptr<CameraConfiguration> CvCapture_libcamera_proxy::config;
std::unique_ptr<CameraManager> CvCapture_libcamera_proxy::cm;
std::string CvCapture_libcamera_proxy::cameraId;
std::shared_ptr<Camera> CvCapture_libcamera_proxy::camera;


void CvCapture_libcamera_proxy::cam_init()
{
    
    if(!cameraId.empty()) 
    {
       open(ind);
    }
}

void CvCapture_libcamera_proxy::cam_init(int index)
{
    if(!cameraId.empty()) 
    {
       open(index);
    }
    
}

void CvCapture_libcamera_proxy::requestComplete(Request *request)
{
	if (request->status() == Request::RequestCancelled)
    {
		return;
    }

	processRequest(request);
}

void CvCapture_libcamera_proxy::processRequest(Request *request)
{
	std::cerr<< "Request completed: " << request->toString() << std::endl;

    const Request::BufferMap &buffers = request->buffers();
	for (auto bufferPair : buffers) 
    {
		// (Unused) Stream *stream = bufferPair.first;
		FrameBuffer *buffer = bufferPair.second;
		const FrameMetadata &metadata = buffer->metadata();

		std::cout << std::endl;
    }
	request->reuse(Request::ReuseBuffers);
	camera->queueRequest(request);
}


bool CvCapture_libcamera_proxy::open(int index)
{
    std::cerr<<"Entered Open";
    try
    {
        cameraId = cm->cameras()[index]->id();
        camera=cm->get(cameraId);
        camera->acquire();

        config = camera->generateConfiguration( { StreamRole::VideoRecording } );

        config->validate();
        streamConfig = config->at(index);
	    std::cout << "Validated viewfinder configuration is: "
		  << streamConfig.toString() << std::endl;
	    camera->configure(config.get());

        opened = true;
        // return true;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        std::cout<<"CvCapture_libcamera_proxy failed";
        opened=false;
    }
    return opened;
}

bool CvCapture_libcamera_proxy::grabFrame()
{
    if(opened==false)
    {
        open(0);
    }
    FrameBufferAllocator *allocator = new FrameBufferAllocator(camera);

	for (StreamConfiguration &cfg : *config) 
    {
		int ret = allocator->allocate(cfg.stream());
		if (ret < 0) 
        {
			std::cerr << "Can't allocate buffers" << std::endl;
			return false;
		}

		size_t allocated = allocator->buffers(cfg.stream()).size();
		std::cout << "Allocated " << allocated << " buffers for stream" << std::endl;
	}
    
    Stream *stream = streamConfig.stream();
    std::cout<<streamConfig.toString()<<std::endl;
	const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator->buffers(stream);
	std::vector<std::unique_ptr<Request>> requests;
	for (unsigned int i = 0; i < buffers.size(); ++i) 
    {
        std::cout<<"Creating Requests";
		std::unique_ptr<Request> request = camera->createRequest();
		if (!request)
		{
			std::cerr << "Can't create request" << std::endl;
			return EXIT_FAILURE;
		}

		const std::unique_ptr<FrameBuffer> &buffer = buffers[i];
		int ret = request->addBuffer(stream, buffer.get());
        std::cout<<"Trying to set buffer";
		if (ret < 0)
		{
			std::cerr << "Can't set buffer for request"
				  << std::endl;
			return EXIT_FAILURE;
		}

		requests.push_back(std::move(request));
    }
    camera->requestCompleted.connect(requestComplete);

    camera->start();
    for (std::unique_ptr<Request> &request : requests)
    {
        std::cout<<"Request getting queued";
     	camera->queueRequest(request.get());
    }

    camera->stop();
	allocator->free(stream);
	delete allocator;
	camera->release();
	camera.reset();
	cm->stop();
 
    return true;
}
// bool CvCapture_libcamera_proxy::grabFrame()
// {

// }


cv::Ptr<cv::IVideoCapture> create_libcamera_capture_cam(int index)
{
    cv::Ptr<CvCapture_libcamera_proxy> capture = cv::makePtr<CvCapture_libcamera_proxy>();

    std::cout << "index passed to create_libcamera_capture_cam : " << index << std::endl;
    if (capture)
        return capture;

    return cv::Ptr<cv::IVideoCapture>();
}

}//namespace cv 
