#include "precomp.hpp"

#include <iostream>
#include <memory>
#include <queue>
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

    bool retrieveFrame(int, OutputArray) CV_OVERRIDE;

    virtual int getCaptureDomain() CV_OVERRIDE { return CAP_LIBCAMERA; }

    ~CvCapture_libcamera_proxy()
    {
        if (opened_)
        {
            camera_->stop();
            allocator.reset();
            camera_->release();
            cm_->stop();
        }
    }

    private:
    int ind;
    StreamConfiguration streamConfig_;
    static std::unique_ptr<CameraConfiguration> config_;
    static std::unique_ptr<CameraManager> cm_;
    static std::shared_ptr<Camera> camera_;
    static std::string cameraId_;
    static std::queue<Request*> completedRequests_;
    bool opened_=false;
    unsigned int allocated_;
    int reqlen_=0;

    protected:
    void cam_init();
    void cam_init(int index);
    std::unique_ptr<Request> request;
    std::vector<std::unique_ptr<Request>> requests;
    std::unique_ptr<FrameBufferAllocator> allocator;
    static void processRequest(Request *request);
    static void requestComplete(Request *request);

};


std::queue<Request*> CvCapture_libcamera_proxy::completedRequests_;
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
    else
    {
        std::cout<<"Request Not Cancelled"<<std::endl;
    }
    processRequest(request);
}

void CvCapture_libcamera_proxy::processRequest(Request *request)
{
    std::cout<<"Entered processRequest"<<std::endl;
	std::cerr<< "Request completed: " << request->toString() << std::endl;
    completedRequests_.push(request);
    std::cout<<"Reusing requests"<<std::endl;
}

bool CvCapture_libcamera_proxy::open(int index)
{
    unsigned int nbuffers = UINT_MAX;
    int ret=0; 
    std::cerr<<"Entered Open";
    try
    {
        cameraId_ = cm_->cameras()[index]->id();
        camera_=cm_->get(cameraId_);
        if (!camera_) 
        { 
            std::cerr << "Camera " << cameraId_ << " not found" << std::endl;
        }
        camera_->acquire();
        config_ = camera_->generateConfiguration( { StreamRole::VideoRecording } );
        config_->validate();
        streamConfig_ = config_->at(index);
	    std::cout << "Validated viewfinder configuration is: "<< streamConfig_.toString() << std::endl;
	    camera_->configure(config_.get());
        allocator = std::make_unique<FrameBufferAllocator>(camera_);
	    for (StreamConfiguration &cfg : *config_) 
        {
            ret = allocator->allocate(cfg.stream());
		    if (ret < 0) 
            {
                std::cerr << "Can't allocate buffers" << std::endl;
			    return false;
		    }

		    allocated_ = allocator->buffers(cfg.stream()).size();
            nbuffers=std::min(nbuffers, allocated_);
		    std::cout << "Allocated " << allocated_ << " buffers for stream" << std::endl;
        }
    
        for (unsigned int i = 0; i < nbuffers; i++) 
        {
            std::cout<<"Creating Requests"<<std::endl;
		    request = camera_->createRequest();
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
                    std::cerr << "Can't set buffer for request"<< std::endl;
				    return ret;
                }
            }
            requests.push_back(std::move(request));
            reqlen_ = requests.size();
            std::cout<<"internal request size: "<<requests.size()<<std::endl;
        }

        
        camera_->requestCompleted.connect(requestComplete);
        camera_->start();
        for (std::unique_ptr<Request> &request : requests)
        {
         	camera_->queueRequest(request.get());
            std::cout<<"Request getting queued"<<std::endl;
        }
        opened_ = true;
        // return true;
    }
    
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        std::cout<<"CvCapture_libcamera_proxy::open failed";
        opened_=false;
    }
    return opened_;
}

bool CvCapture_libcamera_proxy::grabFrame()
{
    if(opened_==false)
    {
        open(0);
    }
 
    return true;
}

bool CvCapture_libcamera_proxy::retrieveFrame(int, OutputArray)
{
    if(completedRequests_.empty())
    {
        return false;
    }
    std::cout<<"retrieveFrame"<<std::endl;
    std::cout<<"Retrieved Frame "<<completedRequests_.front()->sequence()<<std::endl;
    auto nextProcessedRequest = completedRequests_.front();
    completedRequests_.pop();
    //var stores the first value of the queue
    //Mat handling to be done here

    nextProcessedRequest->reuse(Request::ReuseBuffers);
	camera_->queueRequest(nextProcessedRequest);

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
