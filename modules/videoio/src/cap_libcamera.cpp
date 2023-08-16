#include "precomp.hpp"

#include <iostream>
#include <sys/mman.h>
#include <errno.h>
#include <memory>
#include <queue>
#include <map>
#include <unistd.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <libcamera/libcamera.h>
#include <libcamera/framebuffer.h>
#include <libcamera/base/span.h>

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

    Mat convertToRgb(libcamera::Request *req);

    ~CvCapture_libcamera_proxy()
    {
        if (opened_)
        {
            camera_->stop();
            allocator_.reset();
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
    std::vector<libcamera::Span<uint8_t>> planes_;
    std::vector<std::pair<void *, size_t>> maps_;
    std::vector<std::unique_ptr<Request>> requests_;
    std::unique_ptr<FrameBufferAllocator> allocator_;
    bool opened_=false;
    unsigned int allocated_;
    cv::Mat bgrMat_;
    int reqlen_=0;

    protected:
    void cam_init();
    void cam_init(int index);
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
}

Mat CvCapture_libcamera_proxy::convertToRgb(libcamera::Request *req)
{
    cv::Size imageSize;
    auto imgWidth=streamConfig_.size.width;
    auto imgHeight=streamConfig_.size.height;
    std::cout<<"imgWidth: "<<imgWidth<<std::endl;
    std::cout<<"imgHeight: "<<imgHeight<<std::endl;
    imageSize = cv::Size(imgWidth, imgHeight);

    for (const StreamConfiguration &cfg : *config_) 
    {
		const StreamFormats &formats = cfg.formats();
        FrameBuffer *buffer = req->findBuffer(cfg.stream());
        PixelFormat pixFormat=formats.pixelformats()[0];
        std::cout<<pixFormat<<std::endl;

    // CV_ASSERT(!buffer->planes().empty());
	planes_.reserve(buffer->planes().size());

	int mmapFlags = 0;
	mmapFlags |= PROT_READ;
	mmapFlags |= PROT_WRITE;

	struct MappedBufferInfo 
    {
		uint8_t *address = nullptr;
		size_t mapLength = 0;
		size_t dmabufLength = 0;
	};
	std::map<int, MappedBufferInfo> mappedBuffers;

	for (const FrameBuffer::Plane &plane : buffer->planes()) 
    {
		const int fd = plane.fd.get();
		if (mappedBuffers.find(fd) == mappedBuffers.end()) 
        {
			const size_t length = lseek(fd, 0, SEEK_END);
			mappedBuffers[fd] = MappedBufferInfo{ nullptr, 0, length };
		}

		const size_t length = mappedBuffers[fd].dmabufLength;

		if (plane.offset > length || plane.offset + plane.length > length) 
        {
            std::cerr<<"plane is out of buffer"<<std::endl;
		}
		size_t &mapLength = mappedBuffers[fd].mapLength;
		mapLength = std::max(mapLength,static_cast<size_t>(plane.offset + plane.length));
	}
    
    int icount=0;
	for (const FrameBuffer::Plane &plane : buffer->planes()) 
    {
		const int fd = plane.fd.get();
		auto &info = mappedBuffers[fd];
		if (!info.address) 
        {
			void *address = mmap(nullptr, info.mapLength, mmapFlags, MAP_SHARED, fd, 0);
			if (address == MAP_FAILED) 
            {
                std::cerr<<"Failed to mmap plane"<<std::endl;
			}

			info.address = static_cast<uint8_t *>(address);
			maps_.emplace_back(info.address, info.mapLength);
		}

		planes_.emplace_back(info.address + plane.offset, plane.length);
        cv::Mat bgrMat_(imageSize, CV_8U, info.address + plane.offset);
        cv::imdecode(Mat(1, info.mapLength, CV_8U, info.address), IMREAD_COLOR, &bgrMat_);
        icount++;
	}
    }

    // switch(pixFormat)
    // {
    //     case "YUYV":
    //     {
    //         cv::Mat destination(imageSize, CV_8UC3, planes_[i]);
    //         cv::cvtColor(cv::Mat(imageSize, CV_8UC2, planes_[i]), destination, COLOR_YUV2BGR_YUYV);
    //         break;
          
    //     }
    //     case "MJPEG":
    //     {
    //         // CV_LOG_DEBUG(NULL, "VIDEOIO(V4L2:" << deviceName << "): decoding JPEG frame: size=" << currentBuffer.bytesused);
    //         cv::imdecode(Mat(1, currentBuffer.bytesused, CV_8U, start), IMREAD_COLOR, &destination);
    //         break;
    //     }
    // }

    return bgrMat_;
}    

bool CvCapture_libcamera_proxy::open(int index)
{
    std::unique_ptr<Request> request;
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
        allocator_ = std::make_unique<FrameBufferAllocator>(camera_);
	    for (StreamConfiguration &cfg : *config_) 
        {
            ret = allocator_->allocate(cfg.stream());
		    if (ret < 0) 
            {
                std::cerr << "Can't allocate buffers" << std::endl;
			    return false;
		    }

		    allocated_ = allocator_->buffers(cfg.stream()).size();
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
				allocator_->buffers(stream);
			    const std::unique_ptr<FrameBuffer> &buffer = buffers[i];
                std::cout<<"addBuffer"<<std::endl;
			    ret = request->addBuffer(stream, buffer.get());
			    if (ret < 0) 
                {
                    std::cerr << "Can't set buffer for request"<< std::endl;
				    return ret;
                }
            }
            requests_.push_back(std::move(request));
            reqlen_ = requests_.size();
            std::cout<<"internal request size: "<<requests_.size()<<std::endl;requests
        }

        
        camera_->requestCompleted.connect(requestComplete);
        camera_->start();
        for (std::unique_ptr<Request> &req : requests_)
        {
         	camera_->queueRequest(req.get());
            std::cout<<"Request getting queued"<<std::endl;
        }
        opened_ = true;

    }//try
    
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
    std::cout<<opened_<<std::endl;
    if(opened_==false)
    {
        open(0);
    }
 
    return true;
}

bool CvCapture_libcamera_proxy::retrieveFrame(int, OutputArray outputFrame)
{
    if(completedRequests_.empty())
    {
        std::cout<<"completedRequests is empty"<<std::endl;
        // return false;
    }
    std::cout<<"retrieveFrame"<<std::endl;
    std::cout<<"Retrieved Frame "<<completedRequests_.front()->sequence()<<std::endl;
    auto nextProcessedRequest = completedRequests_.front();

    Mat buf = convertToRgb(nextProcessedRequest);
    buf.copyTo(outputFrame);

    completedRequests_.pop();
    return true;
    
/*
    V4L2 comments for reference
    start = (unsigned char*)buffers[MAX_V4L_BUFFERS].memories[MEMORY_ORIG].start; 
    frame.imageData = (char *)buffers[MAX_V4L_BUFFERS].memories[MEMORY_ORIG].start;
    var stores the first value of the queue
    Mat handling to be done here
*/

    std::cout<<"Reusing buffer in processRequest"<<std::endl;
    nextProcessedRequest->reuse(Request::ReuseBuffers);
    std::cout<<"retrieveFrame Request Queued"<<std::endl;
	camera_->queueRequest(nextProcessedRequest);

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
