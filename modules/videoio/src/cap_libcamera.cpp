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
        cm_ = std::make_unique<CameraManager>();
        cm_->start();
        cameraId_ = cm_->cameras()[ind]->id();
        cam_init();
    }

    bool open(int index);

    bool isOpened() const CV_OVERRIDE
    {
       return opened_;
    }

    bool grabFrame() CV_OVERRIDE;

    bool retrieveFrame(int, OutputArray) CV_OVERRIDE
    {
        return false;
    }

    virtual int getCaptureDomain() CV_OVERRIDE { return CAP_LIBCAMERA; }

    private:
    int ind;
    StreamConfiguration streamConfig_;
    static std::unique_ptr<CameraConfiguration> config_;
    static std::unique_ptr<CameraManager> cm_;
    static std::shared_ptr<Camera> camera_;
    static std::string cameraId_;
    bool opened_=false;
    bool grabFrame_check;
    bool grabbedinOpen;
    size_t allocated_;

    protected:
    void cam_init();
    void cam_init(int index);
    static void processRequest(Request *request);
    static void requestComplete(Request *request);

};

std::unique_ptr<CameraConfiguration> CvCapture_libcamera_proxy::config_;
std::unique_ptr<CameraManager> CvCapture_libcamera_proxy::cm_;
std::string CvCapture_libcamera_proxy::cameraId_;
std::shared_ptr<Camera> CvCapture_libcamera_proxy::camera_;


void CvCapture_libcamera_proxy::cam_init()
{
    
    if(!cameraId_.empty()) 
    {
       open(ind);
    }
}

void CvCapture_libcamera_proxy::cam_init(int index)
{
    if(!cameraId_.empty()) 
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
	camera_->queueRequest(request);
}


bool CvCapture_libcamera_proxy::open(int index)
{
    std::cerr<<"Entered Open";
    try
    {
        cameraId_ = cm_->cameras()[index]->id();
        camera_=cm_->get(cameraId_);
        camera_->acquire();

        config_ = camera_->generateConfiguration( { StreamRole::VideoRecording } );

        config_->validate();
        streamConfig_ = config_->at(index);
	    std::cout << "Validated viewfinder configuration is: "
		  << streamConfig_.toString() << std::endl;
	    camera_->configure(config_.get());

        opened_ = true;
        // return true;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        std::cout<<"CvCapture_libcamera_proxy failed";
        opened_=false;
    }
    return opened_;
}

bool CvCapture_libcamera_proxy::grabFrame()
{
    int ret=0;
    if(opened_==false)
    {
        open(0);
    }
    std::unique_ptr<FrameBufferAllocator> allocator;
    // FrameBufferAllocator *allocator = new FrameBufferAllocator(camera);
    allocator = std::make_unique<FrameBufferAllocator>(camera_);
    // Stream *stream;
	for (StreamConfiguration &cfg : *config_) 
    {
		ret = allocator->allocate(cfg.stream());
		if (ret < 0) 
        {
			std::cerr << "Can't allocate buffers" << std::endl;
			return false;
		}

		allocated_ = allocator->buffers(cfg.stream()).size();
		std::cout << "Allocated " << allocated_ << " buffers for stream" << std::endl;
        
	}
    
	std::vector<std::unique_ptr<Request>> requests;
	for (unsigned int i = 0; i < allocated_; ++i) 
    {
        std::cout<<"Creating Requests"<<std::endl;
		std::unique_ptr<Request> request = camera_->createRequest();
		if (!request)
		{
			std::cerr << "Can't create request" << std::endl;
			return EXIT_FAILURE;
		}
    
		for (StreamConfiguration &cfg : *config_) 
        {
			Stream *stream = cfg.stream();
			const std::vector<std::unique_ptr<FrameBuffer>> &buffers =
				allocator->buffers(stream);
			const std::unique_ptr<FrameBuffer> &buffer = buffers[i];
            std::cout<<"addBuffer"<<std::endl;
			ret = request->addBuffer(stream, buffer.get());
			if (ret < 0) 
            {
				std::cerr << "Can't set buffer for request"
					  << std::endl;
				return ret;
			}
        }
		requests.push_back(std::move(request));
    }   
    camera_->requestCompleted.connect(requestComplete);

    camera_->start();
    for (std::unique_ptr<Request> &request : requests)
    {
        std::cout<<"Request getting queued"<<std::endl;
     	camera_->queueRequest(request.get());
    }

    camera_->stop();
	// allocator->free(stream);
	allocator.reset();
	camera_->release();
	camera_.reset();
	cm_->stop();
 
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
