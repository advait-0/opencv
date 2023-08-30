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

void CvCapture_libcamera_proxy::cam_init()
{
    if (!cameraId_.empty()) 
    {
       open(0);
    }
}

void CvCapture_libcamera_proxy::cam_init(int index)
{
    if (!cameraId_.empty()) 
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

    if (buffer->planes().empty()) {
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

int CvCapture_libcamera_proxy::convertToRgb(Request *request, OutputArray &outImage)
{
    int ret;
    cv::Mat destination(streamConfig_.size.height, streamConfig_.size.width, CV_8UC2);
    FrameBuffer *fb = nullptr;

    const Request::BufferMap &buffers = request->buffers();
    for (const auto &[stream, buffer] : buffers) 
    {
        if (stream->configuration().pixelFormat == formats::YUYV)
        {
            fb = buffer;
        }
    }

    ret = mapFrameBuffer(fb);
    if (ret < 0) 
    {
        std::cerr <<  "Failed to mmap buffer: " << std::endl;
        return ret;
    }

    cv::cvtColor(cv::Mat(streamConfig_.size.height, streamConfig_.size.width, CV_8UC2, planes_[0].data()),
                 destination, COLOR_YUV2BGR_YUYV);
    destination.copyTo(outImage);

    return 0;
}

bool CvCapture_libcamera_proxy::open(int index)
{
    std::unique_ptr<Request> request;
    unsigned int nbuffers = UINT_MAX;
    int ret = 0; 
    try
    {
        cameraId_ = cm_->cameras()[index]->id();
        camera_ = cm_->get(cameraId_);
        if (!camera_) 
        { 
            std::cerr << "Camera " << cameraId_ << " not found" << std::endl;
        }
        camera_->acquire();
        config_ = camera_->generateConfiguration( { StreamRole::VideoRecording } );
        config_->at(index).pixelFormat = libcamera::formats::YUYV;
        streamConfig_ = config_->at(index);
        config_->validate();
        std::cout << "Validated viewfinder configuration is: "
		  << streamConfig_.toString() << std::endl;
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
            nbuffers = std::min(nbuffers, allocated_);
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

        opened_ = true;

    }//try
    
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        std::cout<<"CvCapture_libcamera_proxy::open failed";
        opened_ = false;
    }
    return opened_;
}

bool CvCapture_libcamera_proxy::grabFrame()
{
    if (!opened_)
    {
        open(0);
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
        std::cout << "converttoRGB failed" << std::endl;
        return false;
    }
    std::cout<<"Retrieved Frame "<<completedRequests_.front()->sequence()<<std::endl;
    completedRequests_.pop();
    nextProcessedRequest->reuse(Request::ReuseBuffers);
    std::cout<<"retrieveFrame Request Queued"<<std::endl;
    camera_->queueRequest(nextProcessedRequest);

    return true;
}

double CvCapture_libcamera_proxy::getProperty(int property_id) const
{
    switch (property_id)
    {
        // case CV_CAP_PROP_POS_MSEC: ;
        case CV_CAP_PROP_POS_FRAMES: return completedRequests_.front()->sequence();
        // case CV_CAP_PROP_POS_AVI_RATIO: return ;
        case CV_CAP_PROP_FRAME_WIDTH: return streamConfig_.size.width;
        case CV_CAP_PROP_FRAME_HEIGHT: return streamConfig_.size.height;
        // case CV_CAP_PROP_FPS: return ;
        // case CV_CAP_PROP_FOURCC: return ;
    }
    return 0;
}

bool CvCapture_libcamera_proxy::setProperty(int property_id, double value)
{
    handled = false;
    switch (property_id)
    {
        case CV_CAP_PROP_FRAME_WIDTH:
        streamConfig_.size.width = cvRound(value);
        handled = true;
        break;

        case CV_CAP_PROP_FRAME_HEIGHT:
        streamConfig_.size.height = cvRound(value);
        handled = true;
        break;
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
