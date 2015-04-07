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

void April::processImage(cv::Mat& img, cv::Mat& image_gray) {

    // detect April tags (requires a gray scale image)
    cv::cvtColor(img, image_gray, CV_BGR2GRAY);

    vector<AprilTags::TagDetection> detections = tagDetector->extractTags(image_gray);

    // print out each detection
    cout << detections.size() << " tags detected:" << endl;
//    for (int i=0; i<detections.size(); i++) {
////    print_detection(detections[i]);
//    }

    // show the current image including any detections
    for (int i=0; i<detections.size(); i++) {
      // also highlight in the image
      detections[i].draw(img);
    }

    // publish tf for first detection
    publishMarkerTF();
    if (detections.size() > 0)
    {
        // Get transform matrix from marker
        double tag_size = 0.2;
        double fx = 644.50;
        double fy = 339.18;
        double px = 600.9586;
        double py = 244.52;
        Eigen::Matrix4d T;
//        T = detections[0].getRelativeTransform(tag_size, fx, fy, px, py);

        Eigen::Vector3d translation;
        Eigen::Matrix3d rotation;
        detections[0].getRelativeTranslationRotation(tag_size, fx, fy, px, py,
                                                 translation, rotation);

        // Take inverse of transform
        Eigen::Matrix4d inverseT;
        Eigen::Matrix3d rot = T.topLeftCorner(3,3);
        Eigen::Vector3d trans = T.topRightCorner(3,1);

        Eigen::Matrix3d inverseRot = rot.transpose();
        Eigen::Vector3d inverseTrans = -rot*trans;

        inverseT.topLeftCorner(3,3) = inverseRot;
        inverseT.topRightCorner(3,1) = inverseTrans;
        inverseT.row(3) << 0, 0, 0, 1;

        T.topLeftCorner(3,3) = rotation;
        T.topRightCorner(3,1) = translation;
        T.row(3) << 0, 0, 0, 1;

//        img = projectAxis(img, detections[0].homography, trans);

        // publish tf of inverse transform
        publishCameraTF(T);
    }

    imshow("April Tag Detection", img); // OpenCV call

}

void April::publishCameraTF(Eigen::Matrix4d T)
{
    tfScalar m00 = T(0,0); tfScalar m01 = T(0,1); tfScalar m02 = T(0,2);
    tfScalar m10 = T(1,0); tfScalar m11 = T(1,1); tfScalar m12 = T(1,2);
    tfScalar m20 = T(2,0); tfScalar m21 = T(2,1); tfScalar m22 = T(2,2);
    tf::Matrix3x3 rotMat(m00,m01,m02,
                        m10,m11,m12,
                        m20,m21,m22);
    tf::Vector3 transVec(T(0,3), T(1,3), T(2,3));

    tf::Transform transform(rotMat, transVec);

    static tf::TransformBroadcaster br;
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

cv::Mat April::projectAxis(cv::Mat img, Eigen::Matrix3d rot, Eigen::Vector3d trans)
{
    // convert rotation and translation vectors to cv::Mat
    cv::Mat rotMat(3,3,cv::DataType<double>::type);
    rotMat.at<double>(0,0) = rot(0,0);
    rotMat.at<double>(0,1) = rot(0,1);
    rotMat.at<double>(0,2) = rot(0,2);
    rotMat.at<double>(1,0) = rot(1,0);
    rotMat.at<double>(1,1) = rot(1,1);
    rotMat.at<double>(1,2) = rot(1,2);
    rotMat.at<double>(2,0) = rot(2,0);
    rotMat.at<double>(2,1) = rot(2,1);
    rotMat.at<double>(2,2) = rot(2,2);

    cv::Mat transVec(3,1,cv::DataType<double>::type);
    transVec.at<double>(0) = trans(0);
    transVec.at<double>(1) = trans(1);
    transVec.at<double>(2) = trans(2);

    // project axis
    cv::Mat axis(4,1,cv::DataType<cv::Point3f>::type);
    axis.at<cv::Point3f>(0) = (cv::Point3f){0,0,0};
    axis.at<cv::Point3f>(1) = (cv::Point3f){0.1,0,0};
    axis.at<cv::Point3f>(2) = (cv::Point3f){0,0.1,0};
    axis.at<cv::Point3f>(3) = (cv::Point3f){0,0,0.1};

    // Camera matrices from camera calibration
    cv::Mat cameraMatrix(3,3,cv::DataType<double>::type);
    cameraMatrix.at<double>(0,0) = 644.50;
    cameraMatrix.at<double>(0,1) = 0;
    cameraMatrix.at<double>(0,2) = 339.18;
    cameraMatrix.at<double>(1,0) = 0;
    cameraMatrix.at<double>(1,1) = 600.9586;
    cameraMatrix.at<double>(1,2) = 244.52;
    cameraMatrix.at<double>(2,0) = 0;
    cameraMatrix.at<double>(2,1) = 0;
    cameraMatrix.at<double>(2,2) = 1;

    cv::Mat distCoeffs(5,1,cv::DataType<double>::type);
    distCoeffs.at<double>(0) = 0.09386;
    distCoeffs.at<double>(1) = 0.03747;
    distCoeffs.at<double>(2) = 0.0026472;
    distCoeffs.at<double>(3) = 0.00422;
    distCoeffs.at<double>(4) = -0.4924;

    std::vector<cv::Point2f> projectedPoints;
    cv::projectPoints(axis, rotMat, transVec, cameraMatrix, distCoeffs, projectedPoints);

    cv::line(img, projectedPoints[0], projectedPoints[1], cv::Scalar(0,0,255), 2);
    cv::line(img, projectedPoints[0], projectedPoints[2], cv::Scalar(0,255,0), 2);
    cv::line(img, projectedPoints[0], projectedPoints[3], cv::Scalar(255,0,0), 2);

    return img;
}
