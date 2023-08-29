#include <iostream>
#include <opencv2/opencv.hpp>
#include <unistd.h>
using namespace std;

int main()
{
    cv::Mat frame;
    cv::VideoCapture cap(0 ,cv::CAP_LIBCAMERA);
    // cv::Ptr<cv::IVideoCapture> ptr = cap.icap;
    std::string a =cap.getBackendName();

    cout<<"Backend: "<<a<<std::endl;

    if(cap.isOpened()==true)
    {
        cout<<"\nTrue"<<std::endl;
    }
    else
    {
        cout<<"False";
    }

    while(true)
    {
       if (cap.read(frame))
	  imshow("Original Video", frame);

        if (cv::waitKey(1) == 'q') // Press 'q' to exit the loop
          break;
    }
    
    return 0;
}
