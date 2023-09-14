/*
 *  cap_libcamera.cpp
 *  For Video I/O
 *  by Advait Dhamorikar on 27/08/23
 *  advaitdhamorikar[at]gmail[dot]com
 *  Copyright 2023. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "precomp.hpp"
#include "cap_libcamera.hpp"

using namespace cv;
using namespace libcamera;

namespace cv {

std::queue<libcamera::Request*> CvCapture_libcamera_proxy::completedRequests_;

void CvCapture_libcamera_proxy::cam_init(int index)
{
    std::cout<<"Cam init called"<<std::endl;
    cameraId_ = cm_->cameras()[index]->id();
        camera_ = cm_->get(cameraId_);
        if (!camera_) 
        { 
            std::cerr << "Camera " << cameraId_ << " not found" << std::endl;
        }
        opened_ = true;
        camera_->acquire();
}

void CvCapture_libcamera_proxy::requestComplete(Request *request)
{
    if (request->status() == Request::RequestCancelled)
    {
		return;
    }
    else
    {
        completedRequests_.emplace(request);
    }
}

struct MappedBufferInfo 
{
        uint8_t *address = nullptr;
        size_t mapLength = 0;
        size_t dmabufLength = 0;
};

int CvCapture_libcamera_proxy::mapFrameBuffer(const FrameBuffer *buffer)
{
    int error;
    if (buffer->planes().empty()) 
    {
        std::cerr << "Buffer has no planes " << std::endl;
        return -EINVAL;
    }
    maps_.clear();
    planes_.clear();
    planes_.reserve(buffer->planes().size());
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
    for (const FrameBuffer::Plane &plane : buffer->planes()) 
    {
        const int fd = plane.fd.get();
        auto &info = mappedBuffers[fd];
        if (!info.address) 
        {
            void *address = mmap(nullptr, info.mapLength, PROT_READ | PROT_WRITE,
                                 MAP_SHARED, fd, 0);
            if (address == MAP_FAILED) 
            {
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

bool CvCapture_libcamera_proxy::icvSetFrameSize(int _width = 1280, int _height = 720)
{
    std::cout<<"Entered icvSetFrameSize"<<std::endl;
    if (_width > 0)
    {
        width_set = _width;
        std::cout<<"icv Width set value: "<<width_set<<std::endl;
    }
    if (_height > 0)
    {
        height_set = _height;
        std::cout<<"icv Width set value: "<<width_set<<std::endl;
        std::cout<<"icv Height set value: "<<height_set<<std::endl;
    }
    /* two subsequent calls setting WIDTH and HEIGHT will change
       the video size */
    if (width_set <= 0 || height_set <= 0)
    {
        return true;
    }
    else
    {
    width_ = width_set;
    height_ = height_set;
    std::cout<<"icvSetFrame Height:"<<height_<<"\tWidth:"<<width_<<std::endl;
    streamConfig_.size.width = width_;
    streamConfig_.size.height = height_;
    return false;
    }
}

int CvCapture_libcamera_proxy::convertToRgb(Request *request, OutputArray &outImage)
{
    int ret;
    cv::Mat destination(streamConfig_.size.height, streamConfig_.size.width, CV_8UC3);
    FrameBuffer *fb = nullptr;
    const Request::BufferMap &buffers = request->buffers();
    for (const auto &[stream, buffer] : buffers) 
    {
        if (stream->configuration().pixelFormat == pixelFormat_)
        {
            fb = buffer;
        }
    }
    ret = mapFrameBuffer(fb);
    const FrameMetadata &metadata = fb->metadata();
    if (ret < 0) 
    {
        std::cerr <<  "Failed to mmap buffer: " << std::endl;
        return ret;
    }

    switch (pixFmt_)
    {
        case 0:
        cv::imdecode(cv::Mat(1, metadata.planes()[0].bytesused, CV_8U, planes_[0].data()), IMREAD_COLOR, &destination);
        destination.copyTo(outImage);
        break;

        case 1:
        cv::cvtColor(cv::Mat(streamConfig_.size.height, streamConfig_.size.width, CV_8UC2, planes_[0].data()), destination, COLOR_YUV2BGR_YUYV);
        destination.copyTo(outImage);
        break;

        default:
        cv::imdecode(cv::Mat(1, metadata.planes()[0].bytesused, CV_8U, planes_[0].data()), IMREAD_COLOR, &destination);
        destination.copyTo(outImage);
    }

    return 0;
}

bool CvCapture_libcamera_proxy::open()
{
    std::cout<<"Entered open"<<std::endl;
    pixelFormat_ = libcamera::formats::MJPEG;
    std::unique_ptr<Request> request;
    unsigned int nbuffers = UINT_MAX;
    int ret = 0; 
    try
    {
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
            nbuffers = std::min(nbuffers, allocated_);
        }
        for (unsigned int i = 0; i < nbuffers; i++) 
        {
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
			    ret = request->addBuffer(stream, buffer.get());
			    if (ret < 0) 
                {
                    std::cerr << "Can't set buffer for request"<< std::endl;
				    return ret;
                }
            }
            requests_.push_back(std::move(request));
        }
        completedRequests_ = {};
        camera_->requestCompleted.connect(requestComplete);
        camera_->start();
        for (std::unique_ptr<Request> &req : requests_)
		camera_->queueRequest(req.get());
    }//try
    
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        std::cerr<<"CvCapture_libcamera_proxy::open failed"<<std::endl;
        opened_ = false;
    }
    return opened_;
}

int gc = 0;
bool CvCapture_libcamera_proxy::grabFrame()
{
    if (!opened_ && gc>0)
    {
        open();
    }
    else if(opened_ && gc==0)
    {
        config_ = camera_->generateConfiguration({strcfg_});
        streamConfig_ = config_->at(0);
        std::cout<<"Validating config in grabFrame"<<std::endl;
        std::cout<<strcfg_<<std::endl;
        config_->at(0).size.width = width_;
        config_->at(0).size.height = height_;
        std::cout<<"Config details:"<<config_->at(0).size.width<<std::endl;
        std::cout<<"Config details:"<<streamConfig_.size.width<<std::endl;
        config_->validate();    
        std::cout << "Validated viewfinder configuration is: "
		  << streamConfig_.toString() << std::endl;
	    camera_->configure(config_.get());
        gc++;
        open();
    }
    return true;
}

bool CvCapture_libcamera_proxy::retrieveFrame(int, OutputArray outputFrame)
{
    if (completedRequests_.empty())
    {
        return false;
    }
    auto nextProcessedRequest = completedRequests_.front();
    int ret  = convertToRgb(nextProcessedRequest, outputFrame);
    if (ret < 0) 
    {
        std::cerr << "converttoRGB failed" << std::endl;
        return false;
    }
    completedRequests_.pop();
    nextProcessedRequest->reuse(Request::ReuseBuffers);
    camera_->queueRequest(nextProcessedRequest);
    return true;
}

double CvCapture_libcamera_proxy::getProperty(int property_id) const
{
    switch (property_id)
    {
        case CV_CAP_PROP_POS_FRAMES: return completedRequests_.front()->sequence();
        case CV_CAP_PROP_FRAME_WIDTH: return streamConfig_.size.width;
        case CV_CAP_PROP_FRAME_HEIGHT: return streamConfig_.size.height;
    }
    return 0;
}

bool CvCapture_libcamera_proxy::setProperty(int property_id, double value)
{
    std::cout<<"Entered setProperty"<<std::endl;
    handled = false;
    switch (property_id)
    {
        case CV_CAP_PROP_FRAME_WIDTH:
            return icvSetFrameSize(cvRound(value), 0);
        case CV_CAP_PROP_FRAME_HEIGHT:
            return icvSetFrameSize(0, cvRound(value));
        case CV_CAP_PROP_MODE:
            pixFmt_ = cvRound(value);
            return getLibcameraPixelFormat(value);
        case CV_CAP_PROP_FORMAT:
            propFmt_ = cvRound(value);
            return getCameraConfiguration(value);
    }
    return handled ? true : false; 
}

cv::Ptr<cv::IVideoCapture> create_libcamera_capture_cam(int index)
{
    cv::Ptr<CvCapture_libcamera_proxy> capture = cv::makePtr<CvCapture_libcamera_proxy>(index);
    if (capture)
    {
        return capture;
    }
    return cv::Ptr<cv::IVideoCapture>();
}
}//namespace cv 