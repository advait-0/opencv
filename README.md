# libcamera - OpenCV Integration 
## This branch is the WIP development branch for the libcamera integration into OpenCV

### Resources

* Homepage: <https://opencv.org>
  * Courses: <https://opencv.org/courses>
* Docs: <https://docs.opencv.org/4.x/>
* Q&A forum: <https://forum.opencv.org>
  * previous forum (read only): <http://answers.opencv.org>
* Issue tracking: <https://github.com/opencv/opencv/issues>
* Additional OpenCV functionality: <https://github.com/opencv/opencv_contrib>
* Donate to OpenCV: <https://opencv.org/support/>

This guide assumes you currently have libcamera installed on your device but do not have opencv installed on your device.
1. Clone this repository.
2. Make sure you have [libcamera installed on your device](https://libcamera.org/getting-started.html).
3. Install all the [dependencies required for bulding OpenCV](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html).
4. Checkout the branch `libcamera-final` using the command `git checkout libcamera-final`.
5. Navigate to the cloned opencv directory and create a new build directory using ``mkdir build`` and then ```cd build```.
6. Set up OpenCV build with CMake, this is the configuration I am using
  ```
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_C_EXAMPLES=ON \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D WITH_LIBCAMERA=ON \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D BUILD_EXAMPLES=ON ..
```
7. Use `make` or use `make -j8` if your processor supports more parallel jobs for faster processing.
8. To install use `sudo make install`
9. After you are done with the installation to test the build with libcamera you can use this [sample program](https://paste.debian.net/1303027/)
10. To run the above file you can use ```g++ libcamera_test.cpp -o libcamera_test `pkg-config --cflags --libs opencv4` ```
    To quit the program press `q`


#### What works 
- Capturing frames from cameras sending MJPEG and YUYV frames
- Setting some properties to configure your settings like:
    - cv::CAP_PROP_FRAME_WIDTH - Change the frame width
    - cv::CAP_PROP_FRAME_HEIGHT - Change the frame height
    - cv::CAP_PROP_MODE - Select the PixelFormat (0 - MJPEG, 1 - YUYV, default:MJPEG)
    - cv::CAP_PROP_FORMAT - Select the StreamRole (0 - Raw, 1 - StillCapture, 2 - VideoRecording, 3 - Viewfinder, default:VideoRecording)
- Getting some data using cap.get
    - cv::CAP_PROP_FRAME_WIDTH - Get the current frame width
    - cv::CAP_PROP_FRAME_HEIGHT - Get the current frame height

#### What are the known issues
- `if (cap.read(frame))` has to be used to check without which crashes are observed.
- NV12 support is yet to be tested.
- Multiple camera support is being worked on, at the moment you can only use one camera at a time.
- Please create new issues against this branch to notify me if you encounter an issue.


### Contributing

Please read the [contribution guidelines](https://github.com/opencv/opencv/wiki/How_to_contribute) before starting work on a pull request.
Feel free to fork and contribute your changes.

#### Summary of the guidelines: (These are OpenCV guidelines)

* One pull request per issue;
* Choose the right base branch;
* Include tests and documentation;
* Clean up "oops" commits before submitting;
* Follow the [coding style guide](https://github.com/opencv/opencv/wiki/Coding_Style_Guide).

### Additional Resources

* [Submit your OpenCV-based project](https://form.jotform.com/233105358823151) for inclusion in Community Friday on opencv.org
* [Subscribe to the OpenCV YouTube Channel](http://youtube.com/@opencvofficial) featuring OpenCV Live, an hour-long streaming show
* [Follow OpenCV on LinkedIn](http://linkedin.com/company/opencv/) for daily posts showing the state-of-the-art in computer vision & AI
* [Apply to be an OpenCV Volunteer](https://form.jotform.com/232745316792159) to help organize events and online campaigns as well as amplify them
* [Follow OpenCV on Mastodon](http://mastodon.social/@opencv) in the Fediverse
* [Follow OpenCV on Twitter](https://twitter.com/opencvlive)
* [OpenCV.ai](https://opencv.ai): Computer Vision and AI development services from the OpenCV team.
