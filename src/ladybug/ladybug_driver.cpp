#include <iostream>
#include <string>
#include <sstream>
#include "ladybug.h"
#include "ladybugstream.h"
#include <stdexcept>
#include <unistd.h>
#include <signal.h>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>


using namespace std;

static volatile int running_ = 1;


LadybugContext m_context;
LadybugDataFormat m_dataFormat;
// camera config settings
float m_frameRate, m_shutterTime, m_gainAmount;
bool m_isFrameRateAuto, m_isShutterAuto, m_isGainAuto;
int m_jpegQualityPercentage;

ros::Publisher pub[LADYBUG_NUM_CAMERAS + 1];


/**
 * Callback function when the user requests for shutdown
 * Will signal the main thread to stop grabbing frames
 */
static void signalHandler(int) {
    running_ = 0;
    ros::shutdown();
}



/**
 * Callback when we are loading camera parameter file
 * This will load all the camera K&D into into the camerainfo ROS message
 */
void parseCameraInfo(const cv::Mat  &camMat, const cv::Mat  &disCoeff, const cv::Size &imgSize, sensor_msgs::CameraInfo &msg) {

    // Set the header
    msg.header.frame_id = "camera";
    msg.height = (uint32_t)imgSize.height;
    msg.width  = (uint32_t)imgSize.width;

    // K matrix intrinsics
    for (int row=0; row<3; row++) {
        for (int col=0; col<3; col++) {
            msg.K[row * 3 + col] = camMat.at<double>(row, col);
        }
    }

    // P matrix
    for (int row=0; row<3; row++) {
        for (int col=0; col<4; col++) {
            if (col == 3) {
                msg.P[row * 4 + col] = 0.0f;
            } else {
                msg.P[row * 4 + col] = camMat.at<double>(row, col);
            }
        }
    }

    // D distortion params
    for (int row=0; row<disCoeff.rows; row++) {
        for (int col=0; col<disCoeff.cols; col++) {
            msg.D.push_back(disCoeff.at<double>(row, col));
        }
    }
}


/**
 * This will load the camera intrinsics from file
 * It will load it, parse it, and return the topic
 */
void GetMatricesFromFile(ros::NodeHandle nh, sensor_msgs::CameraInfo &camerainfo_msg, size_t cam_id) {

    //////////////////CAMERA INFO/////////////////////////////////////////
    cv::Mat  cameraExtrinsicMat;
    cv::Mat  cameraMat;
    cv::Mat  distCoeff;
    cv::Size imageSize;
    std::string filename;

    // Load the location of the calibration file
    if (nh.getParam("calib_file_"+std::to_string(cam_id), filename) && filename!="") {
        ROS_INFO("Trying to parse calib_file_%d",(int)cam_id);
        ROS_INFO("> %s", filename.c_str());
    } else {
        ROS_INFO("No calib_file_%d param was received, will not load",(int)cam_id);
        return;
    }

    // Use opencv to load the file
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        ROS_INFO("Cannot open %s", filename.c_str());;
        return;
    } else {
        fs["CameraMat"] >> cameraMat;
        fs["DistCoeff"] >> distCoeff;
        fs["ImageSize"] >> imageSize;
    }

    // Finally, parse the yaml file!
    parseCameraInfo(cameraMat, distCoeff, imageSize, camerainfo_msg);
}



/**
 * This function will publish a given image to the ROS communication framework
 */
void publishImage(ros::Time& timestamp, cv::Mat& image, ros::Publisher& image_pub, long int& count, size_t camid) {

    // Create the message
    sensor_msgs::Image msg;
    msg.header.seq = (uint)count;
    msg.header.frame_id = "camera" + std::to_string(camid);
    msg.header.stamp = timestamp;
    msg.height = (uint)image.size().height;
    msg.width  = (uint)image.size().width;
    msg.encoding = "rgb8";
    msg.step = (uint)(image.cols*image.elemSize());
    size_t image_size = image.rows * image.cols * image.elemSize();

    // Copy the actual image data
    msg.data.resize(image_size);
    memcpy(msg.data.data(), image.data, image_size);

    // Publish
    image_pub.publish(msg);
}



/**
 * This will use the ladybug SDK to initalize the camera
 * We need to first create the context, and detect the cameras attached
 * We then load the properties of the camera and initialize the communication
 */
LadybugError init_camera() {

    // Create the SDK context
    LadybugError error;
    error = ladybugCreateContext(&m_context);
    if (error != LADYBUG_OK) {
        throw std::runtime_error("Unable to create Ladybug context.");
    }

    // Here we want to get the number of cameras this sensor has
    LadybugCameraInfo enumeratedCameras[16];
    unsigned int numCameras = 16;
    error = ladybugBusEnumerateCameras(m_context, enumeratedCameras, &numCameras);
    if (error != LADYBUG_OK) {
        return error;
    }
    ROS_INFO("%d cameras detected",numCameras);

    // If we where not able to load any cameras, then error
    // NOTE: Need to at least have one camera...
    if (numCameras == 0) {
        ROS_ERROR("Insufficient number of cameras detected. ");
        return LADYBUG_FAILED;
    }

    // Finally, lets initalize!
    error = ladybugInitializeFromIndex(m_context, 0);
    if (error != LADYBUG_OK) {
        return error;
    }

    // Get the camera information about the connected device
    LadybugCameraInfo camInfo;
    error = ladybugGetCameraInfo(m_context, &camInfo);
    if (error != LADYBUG_OK) {
        return error;
    }


    // Bunch of maps between the enums of the SDK and strings
    // This allows for nice printing of the properties of the sensor for debug
    std::map<LadybugDeviceType,std::string> map_devicetype;
    map_devicetype.insert({LADYBUG_DEVICE_LADYBUG,"Ladybug 1"});
    map_devicetype.insert({LADYBUG_DEVICE_COMPRESSOR,"Ladybug 2"});
    map_devicetype.insert({LADYBUG_DEVICE_LADYBUG3,"Ladybug 3"});
    map_devicetype.insert({LADYBUG_DEVICE_LADYBUG5,"Ladybug 5"});
    map_devicetype.insert({LADYBUG_DEVICE_LADYBUG5P,"Ladybug 5+"});
    map_devicetype.insert({LADYBUG_DEVICE_UNKNOWN,"Unknown"});
    map_devicetype.insert({LADYBUG_DEVICE_FORCE_QUADLET,"Unknown"});
    std::map<LadybugInterfaceType,std::string> map_interfacetype;
    map_interfacetype.insert({LADYBUG_INTERFACE_IEEE1394,"IEEE1394"});
    map_interfacetype.insert({LADYBUG_INTERFACE_USB2,"USB 2.0"});
    map_interfacetype.insert({LADYBUG_INTERFACE_USB3,"USB 3.0"});
    map_interfacetype.insert({LADYBUG_INTERFACE_UNKNOWN,"Unknown"});
    map_interfacetype.insert({LADYBUG_INTERFACE_FORCE_QUADLET,"Unknown"});
    std::map<LadybugBusSpeed,std::string> map_busspeed;
    map_busspeed.insert({LADYBUG_S100,"100Mb/s"});
    map_busspeed.insert({LADYBUG_S200,"200Mb/s"});
    map_busspeed.insert({LADYBUG_S400,"400Mb/s"});
    map_busspeed.insert({LADYBUG_S800,"800Mb/s"});
    map_busspeed.insert({LADYBUG_S1600,"1.6Gb/s"});
    map_busspeed.insert({LADYBUG_S3200,"3.2Gb/s"});
    map_busspeed.insert({LADYBUG_S_FASTEST,"Fastest Possible"});
    map_busspeed.insert({LADYBUG_SPEED_UNKNOWN,"Unknown"});
    map_busspeed.insert({LADYBUG_SPEED_FORCE_QUADLET,"Unknown"});


    // Debug print the parameters about it
    ROS_INFO("Camera Information:");
    ROS_INFO("\t- Base s/n: %d", camInfo.serialBase);
    ROS_INFO("\t- Head s/n: %d", camInfo.serialHead);
    ROS_INFO("\t- Model: %s", camInfo.pszModelName);
    ROS_INFO("\t- Sensor: %s", camInfo.pszSensorInfo);
    ROS_INFO("\t- Vendor: %s", camInfo.pszVendorName);
    ROS_INFO("\t- Device Type: %s", map_devicetype[camInfo.deviceType].c_str());
    ROS_INFO("\t- Interface Type: %s", map_interfacetype[camInfo.interfaceType].c_str());
    ROS_INFO("\t- Bus / Node: %d, %d", camInfo.iBusNum , camInfo.iNodeNum);
    ROS_INFO("\t- Bus Speed: %s", map_busspeed[camInfo.maxBusSpeed].c_str());


    // Values for each of the different type of ladybug cameras
    switch (camInfo.deviceType) {
        case LADYBUG_DEVICE_LADYBUG3:
        {
            m_dataFormat = LADYBUG_DATAFORMAT_RAW8;
            m_frameRate = 16.0f;
            m_isFrameRateAuto = true;
            m_jpegQualityPercentage = 80;
            m_isShutterAuto = true;
            m_shutterTime = 0.1f;
            m_isGainAuto = true;
            m_gainAmount = 10;
            break;
        }
        case LADYBUG_DEVICE_LADYBUG5:
        {
            m_dataFormat = LADYBUG_DATAFORMAT_RAW8;
            m_frameRate = 10.0f;
            m_isFrameRateAuto = true;
            m_jpegQualityPercentage = 80;
            m_isShutterAuto = true;
            m_shutterTime = 0.1f;
            m_isGainAuto = true;
            m_gainAmount = 10;
            break;
        }
        case LADYBUG_DEVICE_LADYBUG5P:
        {
            m_dataFormat = LADYBUG_DATAFORMAT_RAW8;
            m_frameRate = 30.0f;
            m_isFrameRateAuto = false;
            m_jpegQualityPercentage = 80;
            m_isShutterAuto = false;
            m_shutterTime = 0.5f;
            m_isGainAuto = false;
            m_gainAmount = 10;
            break;
        }
        default:
        {
            ROS_ERROR("Unsupported ladybug device, need to set default values...");
            throw std::runtime_error("Unable find default values.");
            break;
        }
    }

    return error;
}


/**
 * This will configure the camera with our parameters, and start the actual stream
 * We will set the framerate, and JPEG quality here...
 */
LadybugError start_camera() {

    // Start the camera in the "lock" mode where we can unlock and lock to get the image
    LadybugError error;
    error = ladybugStartLockNext(m_context, m_dataFormat);
    if (error != LADYBUG_OK) {
        return error;
    }

    // Set the framerate of the camera
    ROS_INFO("CONFIG: setting framerate of %d (auto = %d)",(int)m_frameRate,(int)m_isFrameRateAuto);
    error = ladybugSetAbsPropertyEx(m_context, LADYBUG_FRAME_RATE, false, true, m_isFrameRateAuto, m_frameRate);
    if (error != LADYBUG_OK) {
        return error;
    }

    // Set the shutter/exposure of the camera
    ROS_INFO("CONFIG: setting shutter time of %.3f (auto = %d)",m_shutterTime,(int)m_isShutterAuto);
    error = ladybugSetAbsPropertyEx(m_context, LADYBUG_SHUTTER, false, true, m_isShutterAuto, m_shutterTime);
    if (error != LADYBUG_OK) {
        return error;
    }

    // Set the gain of the camera
    ROS_INFO("CONFIG: setting gain db of %d (auto = %d)",(int)m_gainAmount,(int)m_isGainAuto);
    error = ladybugSetAbsPropertyEx(m_context, LADYBUG_GAIN, false, true, m_isGainAuto, m_gainAmount);
    if (error != LADYBUG_OK) {
        return error;
    }

    // Set the JPEG quality of the image
    ROS_INFO("CONFIG: setting jpeg quality of %d",(int)m_jpegQualityPercentage);
    error = ladybugSetJPEGQuality(m_context, m_jpegQualityPercentage);
    if (error != LADYBUG_OK) {
        return error;
    }

    // Perform a quick test to make sure images can be successfully acquired
    ROS_INFO("Testing that images can be acquired..");
    for (int i=0; i < 5; i++) {
        LadybugImage tempImage;
        error = ladybugLockNext(m_context, &tempImage);
        ROS_INFO("\t- got image %d",i+1);
    }
    ROS_INFO("Testing successful! All good to stream!");

    // Unlock all the images we have
    error = ladybugUnlockAll(m_context);
    if (error != LADYBUG_OK) {
        return error;
    }

    // Done, return the erro if wehad one
    return error;
}



/**
 * Stop the camera context on program exit
 */
LadybugError stop_camera() {
    const LadybugError cameraError = ladybugStop(m_context);
    if (cameraError != LADYBUG_OK) {
        ROS_ERROR("Error: Unable to stop camera (%s)", ladybugErrorToString(cameraError));
    }
    return cameraError;
}


/**
 * Get the next image
 */
LadybugError acquire_image( LadybugImage& image ) {
    return ladybugLockNext(m_context, &image);
}


/**
 * Unlock the old image
 */
LadybugError unlock_image( unsigned int bufferIndex ) {
    return ladybugUnlock(m_context, bufferIndex);
}



/**
 * Main method, that will startup the camera
 * This will also make all the ROS publishers needed
 */
int main (int argc, char **argv)
{
    ////ROS STUFF
    ros::init(argc, argv, "ladybug_camera");
    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");

    // set our callback for closing
    signal(SIGTERM, signalHandler);

    // Initialize ladybug camera
    const LadybugError grabberInitError = init_camera();
    if (grabberInitError != LADYBUG_OK) {
        ROS_ERROR("Error: Failed to initialize camera (%s). Terminating...", ladybugErrorToString(grabberInitError));
        return EXIT_FAILURE;
    }

    // Read in how much we should scale each image by
    int image_scale = 100;
    if (private_nh.getParam("scale", image_scale) && image_scale>0 && image_scale<=100) {
        ROS_INFO("Ladybug ImageScale > %i%%", image_scale);
    } else {
        ROS_WARN("Ladybug ImageScale scale must be (0,100]. Defaulting to 20 ");
        image_scale = 20;
    }


    // Read in our launch parameters
    private_nh.param<int>("jpeg_percent", m_jpegQualityPercentage, m_jpegQualityPercentage);
    private_nh.param<float>("framerate", m_frameRate, m_frameRate);
    private_nh.param<bool>("use_auto_framerate", m_isFrameRateAuto, m_isFrameRateAuto);
    private_nh.param<float>("shutter_time", m_shutterTime, m_shutterTime);
    private_nh.param<bool>("use_auto_shutter_time", m_isShutterAuto, m_isShutterAuto);
    private_nh.param<float>("gain_amount", m_gainAmount, m_gainAmount);
    private_nh.param<bool>("use_auto_gain", m_isGainAuto, m_isGainAuto);


    // Get the ladybug camera information, also show the debug
    LadybugCameraInfo camInfo;
    if (LADYBUG_OK != ladybugGetCameraInfo(m_context, &camInfo)) {
        ROS_ERROR("Error: Failed to get camera information. Terminating...");
        return EXIT_FAILURE;
    }

    // Start the camera!
    const LadybugError startError = start_camera();
    if (startError != LADYBUG_OK) {
        ROS_ERROR("Error: Failed to start camera (%s). Terminating...", ladybugErrorToString(startError) );
        return EXIT_FAILURE;
    }


    // Get the camera information
    ///////calibration data
    //sensor_msgs::CameraInfo camerainfo_msg;
    //GetMatricesFromFile(private_nh, camerainfo_msg);
    //ros::Publisher camera_info_pub = n.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 1, true);




    // Create the publishers
    ROS_INFO("Successfully started ladybug camera and stream");
    for (int i = 0; i < LADYBUG_NUM_CAMERAS; i++) {
        std::string topic = "/ladybug/camera" + std::to_string(i) + "/image_raw";
        pub[i] = n.advertise<sensor_msgs::Image>(topic, 100);
        ROS_INFO("Publishing.. %s", topic.c_str());
    }


    // Start camera polling loop
    ros::Rate loop_rate(m_frameRate);
    long int count = 0;
    while (running_ && ros::ok()) {

        // Aquire a new image from the device
        LadybugImage currentImage;
        const LadybugError acquisitionError = acquire_image(currentImage);
        if (acquisitionError != LADYBUG_OK) {
            ROS_WARN("Failed to acquire image. Error (%s). Trying to continue..", ladybugErrorToString(acquisitionError) );
            continue;
        }

        // Convert to OpenCV Mat
        // NOTE: receive Bayer Image, convert to Color 3 channels
        cv::Size size(currentImage.uiFullCols, currentImage.uiFullRows);

        // Current timestamp of this image
        ros::Time timestamp = ros::Time::now();

        // For each of the cameras, publish to ROS
        for(size_t i=0; i<LADYBUG_NUM_CAMERAS; i++) {

            // Debug print outs
            //ROS_INFO("image time %.5f",currentImage.timeStamp.ulSeconds+1e-6*currentImage.timeStamp.ulMicroSeconds);

            // Get the raw image, and convert it into the standard RGB image type
            cv::Mat rawImage(size, CV_8UC1, currentImage.pData + (i * size.width*size.height));
            cv::Mat image(size, CV_8UC3);
            cv::cvtColor(rawImage, image, cv::COLOR_BayerBG2RGB);

            // Resize the image based on the specified amount
            cv::resize(image,image,cv::Size(size.width*image_scale/100, size.height*image_scale/100));

            // By default the image is side-ways, so correct for this
            cv::transpose(image, image);
            cv::flip(image, image, 1);

            // Publish the current image!
            // TODO: also publish the camera info here too!
            publishImage(timestamp, image, pub[i], count, i);

        }

        // Unlock the image buffer for this image
        unlock_image(currentImage.uiBufferIndex);

        // Spin, so everything is published, and wait if needed
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }


    // Shutdown, and disconnect camera
    ROS_INFO("Stopping ladybug_camera...");
    stop_camera();


    // Done! :D
    ROS_INFO("ladybug_camera stopped");
    return EXIT_SUCCESS;
}
