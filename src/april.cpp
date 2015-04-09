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

    // Camera matrices from camera calibration
    cameraMatrix = cv::Mat(3,3,cv::DataType<double>::type);
    cameraMatrix.at<double>(0,0) = 644.50;
    cameraMatrix.at<double>(0,1) = 0;
    cameraMatrix.at<double>(0,2) = 339.18;
    cameraMatrix.at<double>(1,0) = 0;
    cameraMatrix.at<double>(1,1) = 600.9586;
    cameraMatrix.at<double>(1,2) = 244.52;
    cameraMatrix.at<double>(2,0) = 0;
    cameraMatrix.at<double>(2,1) = 0;
    cameraMatrix.at<double>(2,2) = 1;

    distCoeffs = cv::Mat(5,1,cv::DataType<double>::type);
    distCoeffs.at<double>(0) = 0.09386;
    distCoeffs.at<double>(1) = 0.03747;
    distCoeffs.at<double>(2) = 0.0026472;
    distCoeffs.at<double>(3) = 0.00422;
    distCoeffs.at<double>(4) = -0.4924;

    rvec = cv::Mat(3,1,cv::DataType<double>::type);
    tvec = cv::Mat(3,1,cv::DataType<double>::type);

    tag_size = 0.172244;
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

    // publish tf for marker origin in world frame
    publishMarkerTF();



    std::vector<cv::Point3f> objPts;
    std::vector<cv::Point2f> imgPts;

    // show the current image including any detections
    for (int i=0; i<detections.size(); i++) {
        // also highlight in the image
        detections[i].draw(img);

        if (detections[i].id == 1)
        {
            double s = tag_size/2.;

            // Get transform matrix from marker
            objPts.push_back(cv::Point3f(-s,-s, 0));
            objPts.push_back(cv::Point3f( s,-s, 0));
            objPts.push_back(cv::Point3f( s, s, 0));
            objPts.push_back(cv::Point3f(-s, s, 0));

            std::pair<float, float> p1 = detections[i].p[0];
            std::pair<float, float> p2 = detections[i].p[1];
            std::pair<float, float> p3 = detections[i].p[2];
            std::pair<float, float> p4 = detections[i].p[3];
            imgPts.push_back(cv::Point2f(p1.first, p1.second));
            imgPts.push_back(cv::Point2f(p2.first, p2.second));
            imgPts.push_back(cv::Point2f(p3.first, p3.second));
            imgPts.push_back(cv::Point2f(p4.first, p4.second));

//            cv::circle(img, cv::Point2f(p2.first, p2.second), 6, cv::Scalar(0,0,255));
        }

        if (detections[i].id == 2)
        {
            double s = tag_size/2.;

            double Off1 = 0.022225; // z offset
            double zOff2 = Off1 + tag_size;

            // Get transform matrix from marker
            objPts.push_back(cv::Point3f(s+Off1,-s, Off1));
            objPts.push_back(cv::Point3f(s+Off1,-s, zOff2));
            objPts.push_back(cv::Point3f(s+Off1, s, zOff2));
            objPts.push_back(cv::Point3f(s+Off1, s, Off1));

            std::pair<float, float> p1 = detections[i].p[0];
            std::pair<float, float> p2 = detections[i].p[1];
            std::pair<float, float> p3 = detections[i].p[2];
            std::pair<float, float> p4 = detections[i].p[3];
            imgPts.push_back(cv::Point2f(p1.first, p1.second));
            imgPts.push_back(cv::Point2f(p2.first, p2.second));
            imgPts.push_back(cv::Point2f(p3.first, p3.second));
            imgPts.push_back(cv::Point2f(p4.first, p4.second));
        }
    }

    if (objPts.size() == 8)
    {
        cv::solvePnP(objPts, imgPts, cameraMatrix, distCoeffs, rvec, tvec);

        img = projectAxis(img);

        // inverse pose estimation to get camera position
        cv::Mat rMat(3,3,cv::DataType<double>::type);
        cv::Mat rMatTrans(3,3,cv::DataType<double>::type);
        cv::Mat tvecCam(3,1,cv::DataType<double>::type);

        cv::Rodrigues(rvec, rMat);
        cv::transpose(rMat, rMatTrans);
        tvecCam = -rMatTrans * tvec;

        publishCameraTF(rMatTrans, tvecCam);
    }

    imshow("April Tag Detection", img); // OpenCV call

}

void April::publishCameraTF(cv::Mat rMat, cv::Mat tVec)
{
    tfScalar m00 = rMat.at<double>(0,0); tfScalar m01 = rMat.at<double>(0,1); tfScalar m02 = rMat.at<double>(0,2);
    tfScalar m10 = rMat.at<double>(1,0); tfScalar m11 = rMat.at<double>(1,1); tfScalar m12 = rMat.at<double>(1,2);
    tfScalar m20 = rMat.at<double>(2,0); tfScalar m21 = rMat.at<double>(2,1); tfScalar m22 = rMat.at<double>(2,2);
    tf::Matrix3x3 rotMat(m00,m01,m02,
                        m10,m11,m12,
                        m20,m21,m22);
    tf::Vector3 transVec (tVec.at<double>(0), tVec.at<double>(1), tVec.at<double>(2));

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

cv::Mat April::projectAxis(cv::Mat img)
{
    // project axis
    cv::Mat axis(4,1,cv::DataType<cv::Point3f>::type);
    axis.at<cv::Point3f>(0) = (cv::Point3f){0,0,0};
    axis.at<cv::Point3f>(1) = (cv::Point3f){0.1,0,0};
    axis.at<cv::Point3f>(2) = (cv::Point3f){0,0.1,0};
    axis.at<cv::Point3f>(3) = (cv::Point3f){0,0,0.1};

    std::vector<cv::Point2f> projectedPoints;
    cv::projectPoints(axis, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);

    cv::line(img, projectedPoints[0], projectedPoints[1], cv::Scalar(0,0,255), 2);
    cv::line(img, projectedPoints[0], projectedPoints[2], cv::Scalar(0,255,0), 2);
    cv::line(img, projectedPoints[0], projectedPoints[3], cv::Scalar(255,0,0), 2);

    return img;
}


