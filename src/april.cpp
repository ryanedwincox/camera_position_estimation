#include "april.h"

April::April()
{
}

void April::setup(ros::NodeHandle nh)
{
    this->nh = nh;

    //Create video capture object
    int cameraNum = 0;
    const char* filename = "/home/pierre/Documents/SSL_Tracking/images/OrcusPortageBayMarker.mp4";
    cap = new cv::VideoCapture(cameraNum);
    if(!cap->isOpened())  // check if we succeeded
    {
        std::cout << "camera not found" << std::endl;
    }

    // create april tag detector
    tagDetector = new AprilTags::TagDetector(AprilTags::tagCodes36h11);
}

// The processing loop where images are retrieved, tags detected,
// and information about detections generated
void April::loop() {

  cv::Mat image;
  cv::Mat image_gray;

  while (true) {

    // capture frame
    *cap >> image;

    processImage(image, image_gray);

    // exit if any key is pressed
    if (cv::waitKey(1) >= 0) break;
  }
}

void April::processImage(cv::Mat& image, cv::Mat& image_gray) {

    // detect April tags (requires a gray scale image)
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);

    vector<AprilTags::TagDetection> detections = tagDetector->extractTags(image_gray);

    // print out each detection
    cout << detections.size() << " tags detected:" << endl;
//    for (int i=0; i<detections.size(); i++) {
////    print_detection(detections[i]);
//    }

    // show the current image including any detections
    for (int i=0; i<detections.size(); i++) {
      // also highlight in the image
      detections[i].draw(image);
    }

    // publish tf for first detection
    publishMarkerTF();
    publishCameraTF(detections[0]);

    imshow("April Tag Detection", image); // OpenCV call

}

void publishCameraTF(AprilTags::TagDetection detection)
{
    double tag_size = 0.3;
    double fx = 644.50;
    double fy = 339.18;
    double px = 600.9586;
    double py = 244.52;
    Eigen::Matrix4d T;
    T = detection.getRelativeTransform(tag_size, fx, fy, px, py);

    tfScalar m00 = T(0,0); tfScalar m01 = T(0,1); tfScalar m02 = T(0,2);
    tfScalar m10 = T(1,0); tfScalar m11 = T(1,1); tfScalar m12 = T(1,2);
    tfScalar m20 = T(2,0); tfScalar m21 = T(2,1); tfScalar m22 = T(2,2);
    tf::Matrix3x3 rotMat(m00,m01,m02,
                        m10,m11,m12,
                        m20,m21,m22);

    static tf::TransformBroadcaster br;
    tf::Transform transform(rotMat, tf::Vector3(T(0), T(1), T(2)));
//    transform.setOrigin(tf::Vector3(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)));
//    tf::Quaternion q;
//    q.setRPY(rvec.at<double>(0), -rvec.at<double>(2), rvec.at<double>(1)); // GBR
//    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "marker", "camera"));
}

// TODO move this to launch file a static publisher
void April::publishMarkerTF()
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0,1,0));
    tf::Quaternion q;
    q.setRPY(3.1415/2, 0, 3.1415/2);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "marker"));
}
