/*
 * YoloObjectDetector.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

// yolo obj_detect detector
#include "darknet_ros/YoloObjectDetector.hpp"

// Check for xServer
#include <X11/Xlib.h>
#include <boost/thread/pthread/shared_mutex.hpp>
#include <std_msgs/String.h>

#ifdef DARKNET_FILE_PATH
std::string darknetFilePath_ = DARKNET_FILE_PATH;
#else
#error Path of darknet repository is not defined in CMakeLists.txt.
#endif


namespace darknet_ros {

char *cfg;
char *weights;
char *data;
char **detectionNames;

YoloObjectDetector::YoloObjectDetector(ros::NodeHandle nh)
    : nodeHandle_(nh),
      imageTransport_(nodeHandle_),
      numClasses_(0),
      classLabels_(0),
      rosBoxes_(0),
      rosBoxCounter_(0)
{
  ROS_INFO("[YoloObjectDetector] Node started.");

  // Read parameters from config file.
  if (!readParameters()) {
    ros::requestShutdown();
  }

  init();
}

YoloObjectDetector::~YoloObjectDetector()
{
  {
    boost::unique_lock<boost::shared_mutex> lockNodeStatus(mutexNodeStatus_);
    isNodeRunning_ = false;
  }
  yoloThread_.join();
}

bool YoloObjectDetector::readParameters()
{
  // Load common parameters.
  nodeHandle_.param("image_view/enable_opencv", viewImage_, false);
  nodeHandle_.param("test_model", testModel_, false);
  nodeHandle_.param("yolo_always_detect", alwaysDetect_, false);
  nodeHandle_.param("image_view/wait_key_delay", waitKeyDelay_, 3);
  nodeHandle_.param("image_view/enable_console_output", enableConsoleOutput_, false);

  // Check if Xserver is running on Linux.
  if (XOpenDisplay(NULL)) {
    // Do nothing!
    ROS_INFO("[YoloObjectDetector] Xserver is running.");
  } else {
    ROS_INFO("[YoloObjectDetector] Xserver is not running.");
    viewImage_ = false;
  }

  // display image when test model enabled
  if (testModel_) {
      viewImage_ = true;
      enableConsoleOutput_ = true;
  }

  // Set vector sizes.
  nodeHandle_.param("yolo_model/detection_classes/names", classLabels_,
                    std::vector<std::string>(0));
  numClasses_ = classLabels_.size();
  rosBoxes_ = std::vector<std::vector<RosBox_> >(numClasses_);
  rosBoxCounter_ = std::vector<int>(numClasses_);

  return true;
}

void YoloObjectDetector::init()
{
  ROS_INFO("[YoloObjectDetector] init().");

  // Initialize deep network of darknet.
  std::string weightsPath;
  std::string configPath;
  std::string dataPath;
  std::string configModel;
  std::string weightsModel;

  // Threshold of obj_detect detection.
  float thresh;
  nodeHandle_.param("yolo_model/threshold/value", thresh, (float) 0.3);

  nodeHandle_.param("action/delay_time", delayTime_, 10);

  // Path to weights file.
  nodeHandle_.param("yolo_model/weight_file/name", weightsModel,
                    std::string("yolov2-tiny.weights"));
  nodeHandle_.param("weights_path", weightsPath, std::string("/default"));
  weightsPath += "/" + weightsModel;
  weights = new char[weightsPath.length() + 1];
  strcpy(weights, weightsPath.c_str());

  // Path to config file.
  nodeHandle_.param("yolo_model/config_file/name", configModel, std::string("yolov2-tiny.cfg"));
  nodeHandle_.param("config_path", configPath, std::string("/default"));
  configPath += "/" + configModel;
  cfg = new char[configPath.length() + 1];
  strcpy(cfg, configPath.c_str());

  // Path to data folder.
  dataPath = darknetFilePath_;
  dataPath += "/data";
  data = new char[dataPath.length() + 1];
  strcpy(data, dataPath.c_str());

  // Get classes.
  detectionNames = (char**) realloc((void*) detectionNames, (numClasses_ + 1) * sizeof(char*));
  for (int i = 0; i < numClasses_; i++) {
    detectionNames[i] = new char[classLabels_[i].length() + 1];
    strcpy(detectionNames[i], classLabels_[i].c_str());
  }

  // Load network.
  ROS_INFO("[YoloObjectDetector] setup network.");
  setupNetwork(cfg, weights, data, thresh, detectionNames, numClasses_,
                0, 0, 1, 0.5, 0, 0, 0, 0);
  ROS_INFO("[YoloObjectDetector] init yolo.");
  yoloThread_ = std::thread(&YoloObjectDetector::yolo, this);

  // Initialize publisher and subscriber.
  std::string cameraTopicName;
  std::string depthTopicName;
  int cameraQueueSize;
  int depthQueueSize;
  std::string objectDetectorTopicName;
  int objectDetectorQueueSize;
  bool objectDetectorLatch;
  std::string boundingBoxesTopicName;
  int boundingBoxesQueueSize;
  bool boundingBoxesLatch;
  std::string detectionImageTopicName;
  int detectionImageQueueSize;
  bool detectionImageLatch;

  nodeHandle_.param("subscribers/camera_reading/topic", cameraTopicName,
                    std::string("/camera/color/image_raw"));
  nodeHandle_.param("subscribers/camera_reading/queue_size", cameraQueueSize, 1);
//  nodeHandle_.param("subscribers/depth_reading/topic", depthTopicName,
//                    std::string("/camera/depth/image_raw"));
//  nodeHandle_.param("subscribers/depth_reading/queue_size", depthQueueSize, 1);
  nodeHandle_.param("publishers/object_detector/topic", objectDetectorTopicName,
                    std::string("found_object"));
  nodeHandle_.param("publishers/object_detector/queue_size", objectDetectorQueueSize, 1);
  nodeHandle_.param("publishers/object_detector/latch", objectDetectorLatch, false);
  nodeHandle_.param("publishers/bounding_boxes/topic", boundingBoxesTopicName,
                    std::string("bounding_boxes"));
  nodeHandle_.param("publishers/bounding_boxes/queue_size", boundingBoxesQueueSize, 1);
  nodeHandle_.param("publishers/bounding_boxes/latch", boundingBoxesLatch, false);
  nodeHandle_.param("publishers/detection_image/topic", detectionImageTopicName,
                    std::string("detection_image"));
  nodeHandle_.param("publishers/detection_image/queue_size", detectionImageQueueSize, 1);
  nodeHandle_.param("publishers/detection_image/latch", detectionImageLatch, true);

//  imageSub_ = new message_filters::Subscriber<sensor_msgs::Image>(nodeHandle_, cameraTopicName, cameraQueueSize);
//  depthSub_ = new message_filters::Subscriber<sensor_msgs::Image>(nodeHandle_, depthTopicName, depthQueueSize);
//  sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *imageSub_, *depthSub_);
//  sync_->registerCallback(boost::bind(&YoloObjectDetector::callback, this, _1, _2));
  imageSubscriber_ = imageTransport_.subscribe(cameraTopicName, cameraQueueSize,
                                               &YoloObjectDetector::cameraCallback, this);
//  imageSubscriber_ = imageTransport_.subscribe(depthTopicName, depthQueueSize,
//                                               &YoloObjectDetector::depthCallback, this);
  objectPublisher_ = nodeHandle_.advertise<std_msgs::Int8>(objectDetectorTopicName,
                                                           objectDetectorQueueSize,
                                                           objectDetectorLatch);
  boundingBoxesPublisher_ = nodeHandle_.advertise<darknet_ros_msgs::BoundingBoxes>(
      boundingBoxesTopicName, boundingBoxesQueueSize, boundingBoxesLatch);
  detectionImagePublisher_ = nodeHandle_.advertise<sensor_msgs::Image>(detectionImageTopicName,
                                                                       detectionImageQueueSize,
                                                                       detectionImageLatch);
  ROS_INFO("subscriber camera is %s", cameraTopicName.data());
  // Action servers.
  std::string checkForObjectsActionName;
  nodeHandle_.param("actions/camera_reading/topic", checkForObjectsActionName,
                    std::string("check_for_objects"));
  checkForObjectsActionServer_.reset(
      new CheckForObjectsActionServer(nodeHandle_, checkForObjectsActionName, false));
  checkForObjectsActionServer_->registerGoalCallback(
      boost::bind(&YoloObjectDetector::checkForObjectsActionGoalCB, this));
  checkForObjectsActionServer_->registerPreemptCallback(
      boost::bind(&YoloObjectDetector::checkForObjectsActionPreemptCB, this));
  checkForObjectsActionServer_->start();
}
/*
void YoloObjectDetector::callback(const sensor_msgs::ImageConstPtr& imageMsg, const sensor_msgs::ImageConstPtr& depthMsg)
{
  // if (isDetecting()) {
  ROS_DEBUG("[YoloObjectDetector] USB image & depth received.");

  cv_bridge::CvImagePtr cam_image, cam_depth;
  //if (depthMsg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
  //  ROS_INFO("encoding: 16FC1\n");
  //}

  try {
    cam_image = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::BGR8);
    cam_depth = cv_bridge::toCvCopy(depthMsg, sensor_msgs::image_encodings::TYPE_32FC1);
    imageHeader_ = imageMsg->header;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (cam_image) {
    {
      boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_);
      camImageCopy_ = cam_image->image.clone();
      depthImageCopy_ = cam_depth->image.clone();
    }
    {
      boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_);
      imageStatus_ = true;
    }
    frameWidth_ = cam_image->image.size().width;
    frameHeight_ = cam_image->image.size().height;
  }
  // }
  return;
}
*/
void YoloObjectDetector::cameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // if (isDetecting()) {
    ROS_DEBUG("[YoloObjectDetector] USB image received.");
//    ROS_INFO("get camera");

    cv_bridge::CvImagePtr cam_image;

    try {
      cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      imageHeader_ = msg->header;
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    if (cam_image) {
      {
        boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_);
        camImageCopy_ = cam_image->image.clone();
      }
      {
        boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_);
        imageStatus_ = true;
      }
      frameWidth_ = cam_image->image.size().width;
      frameHeight_ = cam_image->image.size().height;
    }
  // }
  return;
}
/*
void YoloObjectDetector::depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // if (isDetecting()) {
  ROS_DEBUG("[YoloObjectDetector] USB depth image received.");

  cv_bridge::CvImagePtr cam_image;

  try {
    cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    imageHeader_ = msg->header;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (cam_image) {
    boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_);
    depthImageCopy_ = cam_image->image.clone();
  }
  // }
  return;
}
 */

void YoloObjectDetector::checkForObjectsActionGoalCB()
{
  ROS_DEBUG("[YoloObjectDetector] Start check for objects action.");

  boost::shared_ptr<const darknet_ros_msgs::CheckForObjectsGoal> ActionPtr =
      checkForObjectsActionServer_->acceptNewGoal();
  objName = ActionPtr->obj_name;
  containTime_ = 0;

  return;
}

void YoloObjectDetector::checkForObjectsActionPreemptCB()
{
  ROS_DEBUG("[YoloObjectDetector] Preempt check for objects action.");
  checkForObjectsActionServer_->setPreempted();
}

bool YoloObjectDetector::isCheckingForObjects() const
{
  return (ros::ok() && checkForObjectsActionServer_->isActive()
      && !checkForObjectsActionServer_->isPreemptRequested());
}

bool YoloObjectDetector::isDetecting() const
{
//  ROS_INFO("test isDetecting");
  return testModel_ || alwaysDetect_ || isCheckingForObjects();
}

bool YoloObjectDetector::publishDetectionImage(const cv::Mat& detectionImage)
{
  if (detectionImagePublisher_.getNumSubscribers() < 1)
    return false;
  cv_bridge::CvImage cvImage;
  cvImage.header.stamp = ros::Time::now();
  cvImage.header.frame_id = "detection_image";
  cvImage.encoding = sensor_msgs::image_encodings::BGR8;
  cvImage.image = detectionImage;
  detectionImagePublisher_.publish(*cvImage.toImageMsg());
  ROS_DEBUG("Detection image has been published.");
  return true;
}

// double YoloObjectDetector::getWallTime()
// {
//   struct timeval time;
//   if (gettimeofday(&time, NULL)) {
//     return 0;
//   }
//   return (double) time.tv_sec + (double) time.tv_usec * .000001;
// }

int YoloObjectDetector::sizeNetwork(network *net)
{
  int i;
  int count = 0;
  for(i = 0; i < net->n; ++i){
    layer l = net->layers[i];
    if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
      count += l.outputs;
    }
  }
  return count;
}

void YoloObjectDetector::rememberNetwork(network *net)
{
  int i;
  int count = 0;
  for(i = 0; i < net->n; ++i){
    layer l = net->layers[i];
    if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
      memcpy(predictions_[demoIndex_] + count, net->layers[i].output, sizeof(float) * l.outputs);
      count += l.outputs;
    }
  }
}

detection *YoloObjectDetector::avgPredictions(network *net, int *nboxes)
{
  int i, j;
  int count = 0;
  fill_cpu(demoTotal_, 0, avg_, 1);
  for(j = 0; j < demoFrame_; ++j){
    axpy_cpu(demoTotal_, 1./demoFrame_, predictions_[j], 1, avg_, 1);
  }
  for(i = 0; i < net->n; ++i){
    layer l = net->layers[i];
    if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
      memcpy(l.output, avg_ + count, sizeof(float) * l.outputs);
      count += l.outputs;
    }
  }
  detection *dets = get_network_boxes(net, buff_[0].w, buff_[0].h, demoThresh_, demoHier_, 0, 1, nboxes);
  return dets;
}

void *YoloObjectDetector::detectInThread()
{

//    ROS_INFO("START DETECT") ;
    running_ = 1;
    float nms = .4;

    layer l = net_->layers[net_->n - 1];
    if (detect_start == 0){
        detect_start += 1;
        float *X = buffLetter_[(buffIndex_ + 2) % 3].data;
        // 不预测会让线程死掉
        float *prediction = network_predict(net_, X);
    }
//    // 不预测会让线程死掉
//    float *prediction = network_predict(net_, X);
    if (isDetecting()) {

        float *X = buffLetter_[(buffIndex_ + 2) % 3].data;
        // 不预测会让线程死掉
        float *prediction = network_predict(net_, X);
        rememberNetwork(net_);
        detection *dets = 0;
        int nboxes = 0;

        dets = avgPredictions(net_, &nboxes);

        if (nms > 0) do_nms_obj(dets, nboxes, l.classes, nms);

        if (enableConsoleOutput_) {
          printf("\033[2J");
          printf("\033[1;1H");
          printf("\nFPS:%.1f\n", fps_);
          printf("Objects:\n\n");
        }
        image display = buff_[(buffIndex_ + 2) % 3];
        draw_detections(display, dets, nboxes, demoThresh_, demoNames_, demoAlphabet_, demoClasses_);

        // extract the bounding boxes and send them to ROS
        int i, j;
        int count = 0;
        for (i = 0; i < nboxes; ++i) {
          float xmin = dets[i].bbox.x - dets[i].bbox.w / 2.;
          float xmax = dets[i].bbox.x + dets[i].bbox.w / 2.;
          float ymin = dets[i].bbox.y - dets[i].bbox.h / 2.;
          float ymax = dets[i].bbox.y + dets[i].bbox.h / 2.;

          if (xmin < 0)
            xmin = 0;
          if (ymin < 0)
            ymin = 0;
          if (xmax > 1)
            xmax = 1;
          if (ymax > 1)
            ymax = 1;

          // iterate through possible boxes and collect the bounding boxes
          for (j = 0; j < demoClasses_; ++j) {
            if (dets[i].prob[j]) {
              float x_center = (xmin + xmax) / 2;
              float y_center = (ymin + ymax) / 2;
              float BoundingBox_width = xmax - xmin;
              float BoundingBox_height = ymax - ymin;

              // define bounding box
              // BoundingBox must be 1% size of frame (3.2x2.4 pixels)
              if (BoundingBox_width > 0.01 && BoundingBox_height > 0.01) {
                roiBoxes_[count].x = x_center;
                roiBoxes_[count].y = y_center;
                roiBoxes_[count].w = BoundingBox_width;
                roiBoxes_[count].h = BoundingBox_height;
                roiBoxes_[count].Class = j;
                roiBoxes_[count].prob = dets[i].prob[j];
                count++;
              }
            }
          }
        }

        // create array to store found bounding boxes
        // if no obj_detect detected, make sure that ROS knows that num = 0
        if (count == 0) {
          roiBoxes_[0].num = 0;
        } else {
          roiBoxes_[0].num = count;
        }

        free_detections(dets, nboxes);
        demoIndex_ = (demoIndex_ + 1) % demoFrame_;

    }
    running_ = 0;
    return 0;
}

void *YoloObjectDetector::fetchInThread()
{
  // if (isDetecting()) {
    IplImage* ROS_img = getIplImage();
    ipl_into_image(ROS_img, buff_[buffIndex_]);
    {
      boost::shared_lock<boost::shared_mutex> lock(mutexImageCallback_);
      buffId_[buffIndex_] = actionId_;
    }
    rgbgr_image(buff_[buffIndex_]);
    letterbox_image_into(buff_[buffIndex_], net_->w, net_->h, buffLetter_[buffIndex_]);
  // }
  return 0;
}

void *YoloObjectDetector::displayInThread(void *ptr)
{
  show_image_cv(buff_[(buffIndex_ + 1)%3], "YOLO V3", ipl_);
  int c = cvWaitKey(waitKeyDelay_);
  if (c != -1) c = c%256;
  if (c == 27) {
      demoDone_ = 1;
      return 0;
  } else if (c == 82) {
      demoThresh_ += .02;
  } else if (c == 84) {
      demoThresh_ -= .02;
      if(demoThresh_ <= .02) demoThresh_ = .02;
  } else if (c == 83) {
      demoHier_ += .02;
  } else if (c == 81) {
      demoHier_ -= .02;
      if(demoHier_ <= .0) demoHier_ = .0;
  }
  return 0;
}

void YoloObjectDetector::setupNetwork(char *cfgfile, char *weightfile, char *datafile, float thresh,
                                      char **names, int classes,
                                      int delay, char *prefix, int avg_frames, float hier, int w, int h,
                                      int frames, int fullscreen)
{
  demoPrefix_ = prefix;
  demoDelay_ = delay;
  demoFrame_ = avg_frames;
  image **alphabet = load_alphabet_with_file(datafile);
  demoNames_ = names;
  demoAlphabet_ = alphabet;
  demoClasses_ = classes;
  demoThresh_ = thresh;
  demoHier_ = hier;
  fullScreen_ = fullscreen;
  net_ = load_network(cfgfile, weightfile, 0);
  // 设置成检测模式，batch从32到1
  set_batch_network(net_, 1);
}

void YoloObjectDetector::yolo()
{
  const auto wait_duration = std::chrono::milliseconds(2000);
  while (!getImageStatus()) {
    printf("Waiting for image.\n");
    if (!isNodeRunning()) {
      return;
    }
    std::this_thread::sleep_for(wait_duration);
  }

  std::thread detect_thread;
  std::thread fetch_thread;

  srand(2222222);

  int i;
  demoTotal_ = sizeNetwork(net_);
  predictions_ = (float **) calloc(demoFrame_, sizeof(float*));
  for (i = 0; i < demoFrame_; ++i){
      predictions_[i] = (float *) calloc(demoTotal_, sizeof(float));
  }
  avg_ = (float *) calloc(demoTotal_, sizeof(float));

  layer l = net_->layers[net_->n - 1];
  roiBoxes_ = (darknet_ros::RosBox_ *) calloc(l.w * l.h * l.n, sizeof(darknet_ros::RosBox_));

  IplImage* ROS_img = getIplImage();
  buff_[0] = ipl_to_image(ROS_img);
  buff_[1] = copy_image(buff_[0]);
  buff_[2] = copy_image(buff_[0]);
  buffLetter_[0] = letterbox_image(buff_[0], net_->w, net_->h);
  buffLetter_[1] = letterbox_image(buff_[0], net_->w, net_->h);
  buffLetter_[2] = letterbox_image(buff_[0], net_->w, net_->h);
  ipl_ = cvCreateImage(cvSize(buff_[0].w, buff_[0].h), IPL_DEPTH_8U, buff_[0].c);

  int count = 0;

  if (!demoPrefix_ && viewImage_) {
    cvNamedWindow("YOLO V3", CV_WINDOW_NORMAL);
    if (fullScreen_) {
      cvSetWindowProperty("YOLO V3", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    } else {
      cvMoveWindow("YOLO V3", 0, 0);
      cvResizeWindow("YOLO V3", 640, 480);
    }
  }

  demoTime_ = what_time_is_it_now();

  while (!demoDone_) {
    buffIndex_ = (buffIndex_ + 1) % 3;
    fetch_thread = std::thread(&YoloObjectDetector::fetchInThread, this);
    detect_thread = std::thread(&YoloObjectDetector::detectInThread, this);
    if (!demoPrefix_) {
      fps_ = 1./(what_time_is_it_now() - demoTime_);
      demoTime_ = what_time_is_it_now();
      if (viewImage_) {
        displayInThread(0);
      }
      publishInThread();
    } else {
      char name[256];
      sprintf(name, "%s_%08d", demoPrefix_, count);
      save_image(buff_[(buffIndex_ + 1) % 3], name);
    }
    fetch_thread.join();
    detect_thread.join();
    ++count;
    if (!isNodeRunning()) {
      demoDone_ = true;
    }
  }

}

IplImage* YoloObjectDetector::getIplImage()
{
  boost::shared_lock<boost::shared_mutex> lock(mutexImageCallback_);
  IplImage* ROS_img = new IplImage(camImageCopy_);
  return ROS_img;
}

bool YoloObjectDetector::getImageStatus(void)
{
  boost::shared_lock<boost::shared_mutex> lock(mutexImageStatus_);
  return imageStatus_;
}

bool YoloObjectDetector::isNodeRunning(void)
{
  boost::shared_lock<boost::shared_mutex> lock(mutexNodeStatus_);
  return isNodeRunning_;
}

void *YoloObjectDetector::publishInThread()
{
  // Publish image.
  //cv::Mat cvImage = cv::cvarrToMat(ipl_);
  //if (!publishDetectionImage(cv::Mat(cvImage))) {
  //  ROS_DEBUG("Detection image has not been broadcasted.");
  //}
  if (isDetecting()) {
    ROS_INFO("START PUBLISH");
    ROS_DEBUG("[YoloObjectDetector] check for objects in image.");
    darknet_ros_msgs::CheckForObjectsResult objectsActionResult;
    cv_bridge::CvImage cvBridgeImage, cvBridgeImageDepth;
    cvBridgeImage.header.stamp = ros::Time::now();
    cvBridgeImage.header.frame_id = "detection_image";
    cvBridgeImage.encoding = sensor_msgs::image_encodings::BGR8;
    cvBridgeImage.image = camImageCopy_;
/*
    cvBridgeImageDepth.header.stamp = ros::Time::now();
    cvBridgeImageDepth.header.frame_id = "detection_image";
    cvBridgeImageDepth.encoding = sensor_msgs::image_encodings::BGR8;
    cvBridgeImageDepth.image = depthImageCopy_;
*/

    // Publish bounding boxes and detection result.
    int num = roiBoxes_[0].num;
    if (num > 0 && num <= 100) {
      for (int i = 0; i < num; i++) {
        for (int j = 0; j < numClasses_; j++) {
          if (roiBoxes_[i].Class == j) {
            rosBoxes_[j].push_back(roiBoxes_[i]);
            rosBoxCounter_[j]++;
          }
        }
      }

      std_msgs::Int8 msg;
      msg.data = num;
      objectPublisher_.publish(msg);

      for (int i = 0; i < numClasses_; i++) {
        if (rosBoxCounter_[i] > 0) {
          darknet_ros_msgs::BoundingBox boundingBox;

          for (int j = 0; j < rosBoxCounter_[i]; j++) {
            int xmin = (rosBoxes_[i][j].x - rosBoxes_[i][j].w / 2) * frameWidth_;
            int ymin = (rosBoxes_[i][j].y - rosBoxes_[i][j].h / 2) * frameHeight_;
            int xmax = (rosBoxes_[i][j].x + rosBoxes_[i][j].w / 2) * frameWidth_;
            int ymax = (rosBoxes_[i][j].y + rosBoxes_[i][j].h / 2) * frameHeight_;

            boundingBox.Class = classLabels_[i];
            boundingBox.probability = rosBoxes_[i][j].prob;
            boundingBox.xmin = xmin;
            boundingBox.ymin = ymin;
            boundingBox.xmax = xmax;
            boundingBox.ymax = ymax;
            boundingBoxesResults_.bounding_boxes.push_back(boundingBox);
          }
        }
      }
      boundingBoxesResults_.header.stamp = ros::Time::now();
      boundingBoxesResults_.header.frame_id = "detection";
      boundingBoxesResults_.image_header = imageHeader_;
      boundingBoxesResults_.image = *(cvBridgeImage.toImageMsg());
//      boundingBoxesResults_.depth = *(cvBridgeImageDepth.toImageMsg());

      if (alwaysDetect_) {
        boundingBoxesPublisher_.publish(boundingBoxesResults_);
      } else {
        bool boxesContainObjName = false;
        for (auto &box : boundingBoxesResults_.bounding_boxes) {
          if (box.Class == objName.data) {
            boxesContainObjName = True;
          }
        }
        if (boxesContainObjName) {
            containTime_++;
            if (containTime_ > delayTime_) {
              objectsActionResult.bounding_boxes = boundingBoxesResults_;
              checkForObjectsActionServer_->setSucceeded(objectsActionResult, "Send bounding boxes.");
            }
        }
      }

    }
    boundingBoxesResults_.bounding_boxes.clear();
    for (int i = 0; i < numClasses_; i++) {
      rosBoxes_[i].clear();
      rosBoxCounter_[i] = 0;
    }
  }

  return 0;
}


} /* namespace darknet_ros*/
