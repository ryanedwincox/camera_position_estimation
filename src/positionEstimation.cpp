#include <iostream>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/imgproc/imgproc.hpp>

#include "april.h"

using namespace std;

int main(int argc, char *argv[])
{
    cout << "hello world" << endl;

    ros::init(argc, argv, "marker_tf_broadcaster");
    ros::NodeHandle nh;

    April april;

    april.setup(nh);

    april.loop();

}
