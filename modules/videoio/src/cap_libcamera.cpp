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

namespace cv {

class CvCapture_libcamera_proxy CV_FINAL : public cv::IVideoCapture
{
public:
   CvCapture_libcamera_proxy(int index = 0)
    {
        ind = 1;
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
    int mapFrameBuffer(const FrameBuffer *buffer);
    int convertToRgb(libcamera::Request *req, OutputArray &outImage);

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
    static void requestComplete(Request *request);
    int ind;
    StreamConfiguration streamConfig_;
    std::unique_ptr<CameraConfiguration> config_;
    std::unique_ptr<CameraManager> cm_;
    std::shared_ptr<Camera> camera_;
    static std::queue<Request*> completedRequests_;
    std::string cameraId_;
    std::vector<libcamera::Span<uint8_t>> planes_;
    std::vector<libcamera::Span<uint8_t>> maps_;
    std::vector<std::unique_ptr<Request>> requests_;
    std::unique_ptr<FrameBufferAllocator> allocator_;
    bool opened_=false;
    unsigned int allocated_;
    int reqlen_=0;

    protected:
    void cam_init();
    void cam_init(int index);
};

std::queue<Request*> CvCapture_libcamera_proxy::completedRequests_;

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
    if(request->status() == Request::RequestCancelled)
    {
		return;
    }
    else
    {
        completedRequests_.emplace(request);
    }
}

struct MappedBufferInfo {
        uint8_t *address = nullptr;
        size_t mapLength = 0;
        size_t dmabufLength = 0;
};

int CvCapture_libcamera_proxy::mapFrameBuffer(const FrameBuffer *buffer)
{
    int error;

    if (buffer->planes().empty()) {
        std::cerr << "Buffer has no planes " << std::endl;
        return -EINVAL;
    }

    maps_.clear();
    planes_.clear();

    planes_.reserve(buffer->planes().size());

    std::map<int, MappedBufferInfo> mappedBuffers;

    for (const FrameBuffer::Plane &plane : buffer->planes()) {
        const int fd = plane.fd.get();
        if (mappedBuffers.find(fd) == mappedBuffers.end()) {
            const size_t length = lseek(fd, 0, SEEK_END);
            mappedBuffers[fd] = MappedBufferInfo{ nullptr, 0, length };
        }

        const size_t length = mappedBuffers[fd].dmabufLength;

        if (plane.offset > length || plane.offset + plane.length > length) {
            std::cerr << "plane is out of buffer: "
                      << "buffer length=" << length
                      << ", plane offset=" << plane.offset
                      << ", plane length=" << plane.length << std::endl;
            return -ERANGE;
        }

        size_t &mapLength = mappedBuffers[fd].mapLength;
        mapLength = std::max(mapLength,
                             static_cast<size_t>(plane.offset + plane.length));
    }

    for (const FrameBuffer::Plane &plane : buffer->planes()) {
        const int fd = plane.fd.get();
        auto &info = mappedBuffers[fd];
        if (!info.address) {
            void *address = mmap(nullptr, info.mapLength, PROT_READ | PROT_WRITE,
                                 MAP_SHARED, fd, 0);
            if (address == MAP_FAILED) {
                error = -errno;
                std::cerr <<  "Failed to mmap plane: "
                          << strerror(-error) << std::endl;
                return -error;
            }

            info.address = static_cast<uint8_t *>(address);
            maps_.emplace_back(info.address, info.mapLength);
        }

        planes_.emplace_back(info.address + plane.offset, plane.length);
    }

    return 0;
}

int CvCapture_libcamera_proxy::convertToRgb(Request *request, OutputArray &outImage)
{
    int ret;
    cv::Mat destination;
    FrameBuffer *fb = nullptr;

    const Request::BufferMap &buffers = request->buffers();
    for (const auto &[stream, buffer] : buffers) {
        if (stream->configuration().pixelFormat == formats::YUYV)
            fb = buffer;
    }

    ret = mapFrameBuffer(fb);
    if (ret < 0) {
        std::cerr <<  "Failed to mmap buffer: " << std::endl;
        return ret;
    }

    cv::cvtColor(cv::Mat(streamConfig_.size.width, streamConfig_.size.height, CV_8UC2, planes_[0].data()),
                 destination, COLOR_YUV2BGR_YUYV);
    destination.copyTo(outImage);

    return 0;
}

bool CvCapture_libcamera_proxy::open(int index)
{
    std::unique_ptr<Request> request;
    unsigned int nbuffers = UINT_MAX;
    int ret=0; 

    try
    {
        cameraId_ = cm_->cameras()[index]->id();
        camera_=cm_->get(cameraId_);
        if(!camera_) 
        { 
            std::cerr << "Camera " << cameraId_ << " not found" << std::endl;
        }
        camera_->acquire();
        config_ = camera_->generateConfiguration( { StreamRole::VideoRecording } );
        config_->at(0).pixelFormat = libcamera::formats::YUYV;
        config_->validate();
        streamConfig_ = config_->at(0);
	    camera_->configure(config_.get());
        allocator_ = std::make_unique<FrameBufferAllocator>(camera_);
	    for (StreamConfiguration &cfg : *config_) 
        {
            ret = allocator_->allocate(cfg.stream());
		    if(ret < 0) 
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
		    if(!request)
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
			    ret = request->addBuffer(stream, buffer.get());
			    if(ret < 0) 
                {
                    std::cerr << "Can't set buffer for request"<< std::endl;
				    return ret;
                }
            }
            requests_.push_back(std::move(request));
            reqlen_ = requests_.size();
        }
        completedRequests_ = {};
        camera_->requestCompleted.connect(requestComplete);
        camera_->start();
        for (std::unique_ptr<Request> &req : requests_)
		camera_->queueRequest(req.get());

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
        open(1);
    }
 
    return true;
}

bool CvCapture_libcamera_proxy::retrieveFrame(int, OutputArray outputFrame)
{
    if(completedRequests_.empty())
    {
        std::cout<<"completedRequests is empty"<<std::endl;
        return false;
    }

    auto nextProcessedRequest = completedRequests_.front();

    int ret  = convertToRgb(nextProcessedRequest, outputFrame);
    if (ret < 0) {
            std::cout << "converttoRGB failed" << std::endl;
            return false;
    }
    //buf.copyTo(outputFrame);

    std::cout<<"Retrieved Frame "<<completedRequests_.front()->sequence()<<std::endl;

    completedRequests_.pop();
    
    
/*
    V4L2 comments for reference
    start = (unsigned char*)buffers[MAX_V4L_BUFFERS].memories[MEMORY_ORIG].start; 
    frame.imageData = (char *)buffers[MAX_V4L_BUFFERS].memories[MEMORY_ORIG].start;
    var stores the first value of the queue
    Mat handling to be done here
*/

    nextProcessedRequest->reuse(Request::ReuseBuffers);
    std::cout<<"retrieveFrame Request Queued"<<std::endl;
    camera_->queueRequest(nextProcessedRequest);

    return true;

}

cv::Ptr<cv::IVideoCapture> create_libcamera_capture_cam(int index)
{
    cv::Ptr<CvCapture_libcamera_proxy> capture = cv::makePtr<CvCapture_libcamera_proxy>();
    std::cout << "index passed to create_libcamera_capture_cam : " << index << std::endl;
    if(capture)
    {
        return capture;
    }

    return cv::Ptr<cv::IVideoCapture>();
}
}//namespace cv 
