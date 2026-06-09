/*
* Copyright (c) 2014, Autonomous Systems Lab
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
* names of its contributors may be used to endorse or promote products
* derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef ROVIO_ROVIONODE_HPP_
#define ROVIO_ROVIONODE_HPP_

#include <memory>
#include <mutex>
#include <queue>
#include <chrono>
#include <iomanip>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

#include <rovio_interfaces/srv/srv_reset_to_pose.hpp>
#include "rovio/RovioFilter.hpp"
#include "rovio/CoordinateTransform/RovioOutput.hpp"
#include "rovio/CoordinateTransform/FeatureOutput.hpp"
#include "rovio/CoordinateTransform/FeatureOutputReadable.hpp"
#include "rovio/CoordinateTransform/YprOutput.hpp"
#include "rovio/CoordinateTransform/LandmarkOutput.hpp"

namespace rovio {

/** \brief Class, defining the Rovio Node
 *
 *  @tparam FILTER  - \ref rovio::RovioFilter
 */
template<typename FILTER>
class RovioNode{
 public:
  // Filter Stuff
  typedef FILTER mtFilter;
  std::shared_ptr<mtFilter> mpFilter_;
  typedef typename mtFilter::mtFilterState mtFilterState;
  typedef typename mtFilterState::mtState mtState;
  typedef typename mtFilter::mtPrediction::mtMeas mtPredictionMeas;
  mtPredictionMeas predictionMeas_;
  typedef typename std::tuple_element<0,typename mtFilter::mtUpdates>::type mtImgUpdate;
  typedef typename mtImgUpdate::mtMeas mtImgMeas;
  mtImgMeas imgUpdateMeas_;
  mtImgUpdate* mpImgUpdate_;
  typedef typename std::tuple_element<1,typename mtFilter::mtUpdates>::type mtPoseUpdate;
  typedef typename mtPoseUpdate::mtMeas mtPoseMeas;
  mtPoseMeas poseUpdateMeas_;
  mtPoseUpdate* mpPoseUpdate_;
  typedef typename std::tuple_element<2,typename mtFilter::mtUpdates>::type mtVelocityUpdate;
  typedef typename mtVelocityUpdate::mtMeas mtVelocityMeas;
  mtVelocityMeas velocityUpdateMeas_;

  struct FilterInitializationState {
    FilterInitializationState()
        : WrWM_(V3D::Zero()),
          state_(State::WaitForInitUsingAccel) {}

    enum class State {
      // Initialize the filter using accelerometer measurement on the next
      // opportunity.
      WaitForInitUsingAccel,
      // Initialize the filter using an external pose on the next opportunity.
      WaitForInitExternalPose,
      // The filter is initialized.
      Initialized
    } state_;

    // Buffer to hold the initial pose that should be set during initialization
    // with the state WaitForInitExternalPose.
    V3D WrWM_;
    QPD qMW_;

    explicit operator bool() const {
      return isInitialized();
    }

    bool isInitialized() const {
      return (state_ == State::Initialized);
    }
  };
  FilterInitializationState init_state_;

  bool forceOdometryPublishing_;
  bool forcePoseWithCovariancePublishing_;
  bool forceTransformPublishing_;
  bool forceExtrinsicsPublishing_;
  bool forceImuBiasPublishing_;
  bool forcePclPublishing_;
  bool forceMarkersPublishing_;
  bool forcePatchPublishing_;
  bool gotFirstMessages_;
  std::mutex m_filter_;

  // Nodes, Subscriber, Publishers
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subImg0_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subImg1_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr subGroundtruth_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subGroundtruthOdometry_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subVelocity_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srvResetFilter_;
  rclcpp::Service<rovio_interfaces::srv::SrvResetToPose>::SharedPtr srvResetToPoseFilter_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdometry_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pubTransform_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pubPoseWithCovStamped_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pub_T_J_W_transform;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tb_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPcl_;       /**<Publisher: Ros point cloud, visualizing the landmarks.*/
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPatch_;     /**<Publisher: Patch data.*/
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubMarkers_; /**<Publisher: Ros line marker, indicating the depth uncertainty of a landmark.*/
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pubExtrinsics_[mtState::nCam_];
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pubImuBias_;

  // Ros Messages
  geometry_msgs::msg::TransformStamped transformMsg_;
  geometry_msgs::msg::TransformStamped T_J_W_Msg_;
  nav_msgs::msg::Odometry odometryMsg_;
  geometry_msgs::msg::PoseWithCovarianceStamped estimatedPoseWithCovarianceStampedMsg_;
  geometry_msgs::msg::PoseWithCovarianceStamped extrinsicsMsg_[mtState::nCam_];
  sensor_msgs::msg::PointCloud2 pclMsg_;
  sensor_msgs::msg::PointCloud2 patchMsg_;
  visualization_msgs::msg::Marker markerMsg_;
  sensor_msgs::msg::Imu imuBiasMsg_;
  int msgSeq_;

  // Rovio outputs and coordinate transformations
  typedef StandardOutput mtOutput;
  mtOutput cameraOutput_;
  MXD cameraOutputCov_;
  mtOutput imuOutput_;
  MXD imuOutputCov_;
  CameraOutputCT<mtState> cameraOutputCT_;
  ImuOutputCT<mtState> imuOutputCT_;
  rovio::TransformFeatureOutputCT<mtState> transformFeatureOutputCT_;
  rovio::LandmarkOutputImuCT<mtState> landmarkOutputImuCT_;
  rovio::FeatureOutput featureOutput_;
  rovio::LandmarkOutput landmarkOutput_;
  MXD featureOutputCov_;
  MXD landmarkOutputCov_;
  rovio::FeatureOutputReadableCT featureOutputReadableCT_;
  rovio::FeatureOutputReadable featureOutputReadable_;
  MXD featureOutputReadableCov_;

  // ROS names for output tf frames.
  std::string map_frame_;
  std::string world_frame_;
  std::string camera_frame_;
  std::string imu_frame_;

  /** \brief Constructor
   */
  RovioNode(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<mtFilter> mpFilter)
      : node_(node), mpFilter_(mpFilter), transformFeatureOutputCT_(&mpFilter->multiCamera_), landmarkOutputImuCT_(&mpFilter->multiCamera_),
        cameraOutputCov_((int)(mtOutput::D_),(int)(mtOutput::D_)), featureOutputCov_((int)(FeatureOutput::D_),(int)(FeatureOutput::D_)), landmarkOutputCov_(3,3),
        featureOutputReadableCov_((int)(FeatureOutputReadable::D_),(int)(FeatureOutputReadable::D_)){
    #ifndef NDEBUG
      RCLCPP_WARN(node_->get_logger(), "====================== Debug Mode ======================");
    #endif
    mpImgUpdate_ = &std::get<0>(mpFilter_->mUpdates_);
    mpPoseUpdate_ = &std::get<1>(mpFilter_->mUpdates_);
    forceOdometryPublishing_ = false;
    forcePoseWithCovariancePublishing_ = false;
    forceTransformPublishing_ = false;
    forceExtrinsicsPublishing_ = false;
    forceImuBiasPublishing_ = false;
    forcePclPublishing_ = false;
    forceMarkersPublishing_ = false;
    forcePatchPublishing_ = false;
    gotFirstMessages_ = false;

    // Subscribe topics
    subImu_ = node_->create_subscription<sensor_msgs::msg::Imu>("imu0", 1000, std::bind(&RovioNode::imuCallback, this, std::placeholders::_1));
    subImg0_ = node_->create_subscription<sensor_msgs::msg::Image>("cam0/image_raw", 1000, std::bind(&RovioNode::imgCallback0, this, std::placeholders::_1));
    subImg1_ = node_->create_subscription<sensor_msgs::msg::Image>("cam1/image_raw", 1000, std::bind(&RovioNode::imgCallback1, this, std::placeholders::_1));
    subGroundtruth_ = node_->create_subscription<geometry_msgs::msg::TransformStamped>("pose", 1000, std::bind(&RovioNode::groundtruthCallback, this, std::placeholders::_1));
    subGroundtruthOdometry_ = node_->create_subscription<nav_msgs::msg::Odometry>("odometry", 1000, std::bind(&RovioNode::groundtruthOdometryCallback, this, std::placeholders::_1));
    subVelocity_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>("abss/twist", 1000, std::bind(&RovioNode::velocityCallback, this, std::placeholders::_1));

    // Initialize ROS service servers
    srvResetFilter_ = node_->create_service<std_srvs::srv::Empty>("rovio/reset", std::bind(&RovioNode::resetServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
    srvResetToPoseFilter_ = node_->create_service<rovio_interfaces::srv::SrvResetToPose>("rovio/reset_to_pose", std::bind(&RovioNode::resetToPoseServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Initialize TF Broadcaster
    tb_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);

    // Advertise topics
    pubTransform_ = node_->create_publisher<geometry_msgs::msg::TransformStamped>("rovio/transform", 1);
    pubOdometry_ = node_->create_publisher<nav_msgs::msg::Odometry>("rovio/odometry", 1);
    pubPoseWithCovStamped_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("rovio/pose_with_covariance_stamped", 1);
    pubPcl_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("rovio/pcl", 1);
    pubPatch_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("rovio/patch", 1);
    pubMarkers_ = node_->create_publisher<visualization_msgs::msg::Marker>("rovio/markers", 1);

    pub_T_J_W_transform = node_->create_publisher<geometry_msgs::msg::TransformStamped>("rovio/T_G_W", 1);
    for(int camID=0;camID<mtState::nCam_;camID++){
      pubExtrinsics_[camID] = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("rovio/extrinsics" + std::to_string(camID), 1);
    }
    pubImuBias_ = node_->create_publisher<sensor_msgs::msg::Imu>("rovio/imu_biases", 1);

    // Handle coordinate frame naming
    map_frame_ = "/map";
    world_frame_ = "/world";
    camera_frame_ = "/camera";
    imu_frame_ = "/imu";
    node_->declare_parameter("map_frame", map_frame_);
    node_->get_parameter("map_frame", map_frame_);
    node_->declare_parameter("world_frame", world_frame_);
    node_->get_parameter("world_frame", world_frame_);
    node_->declare_parameter("camera_frame", camera_frame_);
    node_->get_parameter("camera_frame", camera_frame_);
    node_->declare_parameter("imu_frame", imu_frame_);
    node_->get_parameter("imu_frame", imu_frame_);

    // Initialize messages
    transformMsg_.header.frame_id = world_frame_;
    transformMsg_.child_frame_id = imu_frame_;

    T_J_W_Msg_.child_frame_id = world_frame_;
    T_J_W_Msg_.header.frame_id = map_frame_;

    odometryMsg_.header.frame_id = world_frame_;
    odometryMsg_.child_frame_id = imu_frame_;
    msgSeq_ = 1;
    for(int camID=0;camID<mtState::nCam_;camID++){
      extrinsicsMsg_[camID].header.frame_id = imu_frame_;
    }
    imuBiasMsg_.header.frame_id = world_frame_;
    imuBiasMsg_.orientation.x = 0;
    imuBiasMsg_.orientation.y = 0;
    imuBiasMsg_.orientation.z = 0;
    imuBiasMsg_.orientation.w = 1;
    for(int i=0;i<9;i++){
      imuBiasMsg_.orientation_covariance[i] = 0.0;
    }

    // PointCloud message.
    pclMsg_.header.frame_id = imu_frame_;
    pclMsg_.height = 1;               // Unordered point cloud.
    pclMsg_.width  = mtState::nMax_;  // Number of features/points.
    const int nFieldsPcl = 18;
    std::string namePcl[nFieldsPcl] = {"id","camId","rgb","status","x","y","z","b_x","b_y","b_z","d","c_00","c_01","c_02","c_11","c_12","c_22","c_d"};
    int sizePcl[nFieldsPcl] = {4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4};
    int countPcl[nFieldsPcl] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
    int datatypePcl[nFieldsPcl] = {sensor_msgs::msg::PointField::INT32,sensor_msgs::msg::PointField::INT32,sensor_msgs::msg::PointField::UINT32,sensor_msgs::msg::PointField::UINT32,
        sensor_msgs::msg::PointField::FLOAT32,sensor_msgs::msg::PointField::FLOAT32,sensor_msgs::msg::PointField::FLOAT32,
        sensor_msgs::msg::PointField::FLOAT32,sensor_msgs::msg::PointField::FLOAT32,sensor_msgs::msg::PointField::FLOAT32,sensor_msgs::msg::PointField::FLOAT32,
        sensor_msgs::msg::PointField::FLOAT32,sensor_msgs::msg::PointField::FLOAT32,sensor_msgs::msg::PointField::FLOAT32,sensor_msgs::msg::PointField::FLOAT32,sensor_msgs::msg::PointField::FLOAT32,sensor_msgs::msg::PointField::FLOAT32,sensor_msgs::msg::PointField::FLOAT32};
    pclMsg_.fields.resize(nFieldsPcl);
    int byteCounter = 0;
    for(int i=0;i<nFieldsPcl;i++){
      pclMsg_.fields[i].name     = namePcl[i];
      pclMsg_.fields[i].offset   = byteCounter;
      pclMsg_.fields[i].count    = countPcl[i];
      pclMsg_.fields[i].datatype = datatypePcl[i];
      byteCounter += sizePcl[i]*countPcl[i];
    }
    pclMsg_.point_step = byteCounter;
    pclMsg_.row_step = pclMsg_.point_step * pclMsg_.width;
    pclMsg_.data.resize(pclMsg_.row_step * pclMsg_.height);
    pclMsg_.is_dense = false;

    // PointCloud message.
    patchMsg_.header.frame_id = "";
    patchMsg_.height = 1;               // Unordered point cloud.
    patchMsg_.width  = mtState::nMax_;  // Number of features/points.
    const int nFieldsPatch = 5;
    std::string namePatch[nFieldsPatch] = {"id","patch","dx","dy","error"};
    int sizePatch[nFieldsPatch] = {4,4,4,4,4};
    int countPatch[nFieldsPatch] = {1,mtState::nLevels_*mtState::patchSize_*mtState::patchSize_,mtState::nLevels_*mtState::patchSize_*mtState::patchSize_,mtState::nLevels_*mtState::patchSize_*mtState::patchSize_,mtState::nLevels_*mtState::patchSize_*mtState::patchSize_};
    int datatypePatch[nFieldsPatch] = {sensor_msgs::msg::PointField::INT32,sensor_msgs::msg::PointField::FLOAT32,sensor_msgs::msg::PointField::FLOAT32,sensor_msgs::msg::PointField::FLOAT32,sensor_msgs::msg::PointField::FLOAT32};
    patchMsg_.fields.resize(nFieldsPatch);
    byteCounter = 0;
    for(int i=0;i<nFieldsPatch;i++){
      patchMsg_.fields[i].name     = namePatch[i];
      patchMsg_.fields[i].offset   = byteCounter;
      patchMsg_.fields[i].count    = countPatch[i];
      patchMsg_.fields[i].datatype = datatypePatch[i];
      byteCounter += sizePatch[i]*countPatch[i];
    }
    patchMsg_.point_step = byteCounter;
    patchMsg_.row_step = patchMsg_.point_step * patchMsg_.width;
    patchMsg_.data.resize(patchMsg_.row_step * patchMsg_.height);
    patchMsg_.is_dense = false;

    // Marker message (vizualization of uncertainty)
    markerMsg_.header.frame_id = imu_frame_;
    markerMsg_.id = 0;
    markerMsg_.type = visualization_msgs::msg::Marker::LINE_LIST;
    markerMsg_.action = visualization_msgs::msg::Marker::ADD;
    markerMsg_.pose.position.x = 0;
    markerMsg_.pose.position.y = 0;
    markerMsg_.pose.position.z = 0;
    markerMsg_.pose.orientation.x = 0.0;
    markerMsg_.pose.orientation.y = 0.0;
    markerMsg_.pose.orientation.z = 0.0;
    markerMsg_.pose.orientation.w = 1.0;
    markerMsg_.scale.x = 0.04; // Line width.
    markerMsg_.color.a = 1.0;
    markerMsg_.color.r = 0.0;
    markerMsg_.color.g = 1.0;
    markerMsg_.color.b = 0.0;
  }

  /** \brief Destructor
   */
  virtual ~RovioNode(){}

  /** \brief Tests the functionality of the rovio node.
   *
   *  @todo debug with   doVECalibration = false and depthType = 0
   */
  void makeTest(){
    mtFilterState* mpTestFilterState = new mtFilterState();
    *mpTestFilterState = mpFilter_->init_;
    mpTestFilterState->setCamera(&mpFilter_->multiCamera_);
    mtState& testState = mpTestFilterState->state_;
    unsigned int s = 2;
    testState.setRandom(s);
    predictionMeas_.setRandom(s);
    imgUpdateMeas_.setRandom(s);

    LWF::NormalVectorElement tempNor;
    for(int i=0;i<mtState::nMax_;i++){
      testState.CfP(i).camID_ = 0;
      tempNor.setRandom(s);
      if(tempNor.getVec()(2) < 0){
        tempNor.boxPlus(Eigen::Vector2d(3.14,0),tempNor);
      }
      testState.CfP(i).set_nor(tempNor);
      testState.CfP(i).trackWarping_ = false;
      tempNor.setRandom(s);
      if(tempNor.getVec()(2) < 0){
        tempNor.boxPlus(Eigen::Vector2d(3.14,0),tempNor);
      }
      testState.aux().feaCoorMeas_[i].set_nor(tempNor,true);
      testState.aux().feaCoorMeas_[i].mpCamera_ = &mpFilter_->multiCamera_.cameras_[0];
      testState.aux().feaCoorMeas_[i].camID_ = 0;
    }
    testState.CfP(0).camID_ = mtState::nCam_-1;
    mpTestFilterState->fsm_.setAllCameraPointers();

    // Prediction
    std::cout << "Testing Prediction" << std::endl;
    mpFilter_->mPrediction_.testPredictionJacs(testState,predictionMeas_,1e-8,1e-6,0.1);

    // Update
    if(!mpImgUpdate_->useDirectMethod_){
      std::cout << "Testing Update (can sometimes exhibit large absolut errors due to the float precision)" << std::endl;
      for(int i=0;i<(std::min((int)mtState::nMax_,2));i++){
        testState.aux().activeFeature_ = i;
        testState.aux().activeCameraCounter_ = 0;
        mpImgUpdate_->testUpdateJacs(testState,imgUpdateMeas_,1e-4,1e-5);
        testState.aux().activeCameraCounter_ = mtState::nCam_-1;
        mpImgUpdate_->testUpdateJacs(testState,imgUpdateMeas_,1e-4,1e-5);
      }
    }

    // Testing CameraOutputCF and CameraOutputCF
    std::cout << "Testing cameraOutputCF" << std::endl;
    cameraOutputCT_.testTransformJac(testState,1e-8,1e-6);
    std::cout << "Testing imuOutputCF" << std::endl;
    imuOutputCT_.testTransformJac(testState,1e-8,1e-6);
    std::cout << "Testing attitudeToYprCF" << std::endl;
    rovio::AttitudeToYprCT attitudeToYprCF;
    attitudeToYprCF.testTransformJac(1e-8,1e-6);

    // Testing TransformFeatureOutputCT
    std::cout << "Testing transformFeatureOutputCT" << std::endl;
    transformFeatureOutputCT_.setFeatureID(0);
    if(mtState::nCam_>1){
      transformFeatureOutputCT_.setOutputCameraID(1);
      transformFeatureOutputCT_.testTransformJac(testState,1e-8,1e-5);
    }
    transformFeatureOutputCT_.setOutputCameraID(0);
    transformFeatureOutputCT_.testTransformJac(testState,1e-8,1e-5);

    // Testing LandmarkOutputImuCT
    std::cout << "Testing LandmarkOutputImuCT" << std::endl;
    landmarkOutputImuCT_.setFeatureID(0);
    landmarkOutputImuCT_.testTransformJac(testState,1e-8,1e-5);

    // Getting featureOutput for next tests
    transformFeatureOutputCT_.transformState(testState,featureOutput_);
    if(!featureOutput_.c().isInFront()){
      featureOutput_.c().set_nor(featureOutput_.c().get_nor().rotated(QPD(0.0,1.0,0.0,0.0)),false);
    }

    // Testing FeatureOutputReadableCT
    std::cout << "Testing FeatureOutputReadableCT" << std::endl;
    featureOutputReadableCT_.testTransformJac(featureOutput_,1e-8,1e-5);

    // Testing pixelOutputCT
    rovio::PixelOutputCT pixelOutputCT;
    std::cout << "Testing pixelOutputCT (can sometimes exhibit large absolut errors due to the float precision)" << std::endl;
    pixelOutputCT.testTransformJac(featureOutput_,1e-4,1.0); // Reduces accuracy due to float and strong camera distortion

    // Testing ZeroVelocityUpdate_
    std::cout << "Testing zero velocity update" << std::endl;
    mpImgUpdate_->zeroVelocityUpdate_.testJacs();

    // Testing PoseUpdate
    if(!mpPoseUpdate_->noFeedbackToRovio_){
      std::cout << "Testing pose update" << std::endl;
      mpPoseUpdate_->testUpdateJacs(1e-8,1e-5);
    }

    delete mpTestFilterState;
  }

  /** \brief Callback for IMU-Messages. Adds IMU measurements (as prediction measurements) to the filter.
   */
  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg){
    std::lock_guard<std::mutex> lock(m_filter_);
    predictionMeas_.template get<mtPredictionMeas::_acc>() = Eigen::Vector3d(imu_msg->linear_acceleration.x,imu_msg->linear_acceleration.y,imu_msg->linear_acceleration.z);
    predictionMeas_.template get<mtPredictionMeas::_gyr>() = Eigen::Vector3d(imu_msg->angular_velocity.x,imu_msg->angular_velocity.y,imu_msg->angular_velocity.z);
    if(init_state_.isInitialized()){
      mpFilter_->addPredictionMeas(predictionMeas_,rclcpp::Time(imu_msg->header.stamp).seconds());
      updateAndPublish();
    } else {
      switch(init_state_.state_) {
        case FilterInitializationState::State::WaitForInitExternalPose: {
          std::cout << "-- Filter: Initializing using external pose ..." << std::endl;
          mpFilter_->resetWithPose(init_state_.WrWM_, init_state_.qMW_, rclcpp::Time(imu_msg->header.stamp).seconds());
          break;
        }
        case FilterInitializationState::State::WaitForInitUsingAccel: {
          std::cout << "-- Filter: Initializing using accel. measurement ..." << std::endl;
          mpFilter_->resetWithAccelerometer(predictionMeas_.template get<mtPredictionMeas::_acc>(),rclcpp::Time(imu_msg->header.stamp).seconds());
          break;
        }
        default: {
          std::cout << "Unhandeld initialization type." << std::endl;
          abort();
          break;
        }
      }

      std::cout << std::setprecision(12);
      std::cout << "-- Filter: Initialized at t = " << rclcpp::Time(imu_msg->header.stamp).seconds() << std::endl;
      init_state_.state_ = FilterInitializationState::State::Initialized;
    }
  }

  /** \brief Image callback for the camera with ID 0
   *
   * @param img - Image message.
   * @todo generalize
   */
  void imgCallback0(const sensor_msgs::msg::Image::ConstSharedPtr & img){
    std::lock_guard<std::mutex> lock(m_filter_);
    imgCallback(img,0);
  }

  /** \brief Image callback for the camera with ID 1
   *
   * @param img - Image message.
   * @todo generalize
   */
  void imgCallback1(const sensor_msgs::msg::Image::ConstSharedPtr & img){
    std::lock_guard<std::mutex> lock(m_filter_);
    if(mtState::nCam_ > 1) imgCallback(img,1);
  }

  /** \brief Image callback. Adds images (as update measurements) to the filter.
   *
   *   @param img   - Image message.
   *   @param camID - Camera ID.
   */
  void imgCallback(const sensor_msgs::msg::Image::ConstSharedPtr & img, const int camID = 0){
    // Get image from msg
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_8UC1);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat cv_img;
    cv_ptr->image.copyTo(cv_img);
    if(init_state_.isInitialized() && !cv_img.empty()){
      double msgTime = rclcpp::Time(img->header.stamp).seconds();
      if(msgTime != imgUpdateMeas_.template get<mtImgMeas::_aux>().imgTime_){
        for(int i=0;i<mtState::nCam_;i++){
          if(imgUpdateMeas_.template get<mtImgMeas::_aux>().isValidPyr_[i]){
            std::cout << "    \033[31mFailed Synchronization of Camera Frames, t = " << msgTime << "\033[0m" << std::endl;
          }
        }
        imgUpdateMeas_.template get<mtImgMeas::_aux>().reset(msgTime);
      }
      imgUpdateMeas_.template get<mtImgMeas::_aux>().pyr_[camID].computeFromImage(cv_img,true);
      imgUpdateMeas_.template get<mtImgMeas::_aux>().isValidPyr_[camID] = true;

      if(imgUpdateMeas_.template get<mtImgMeas::_aux>().areAllValid()){
        mpFilter_->template addUpdateMeas<0>(imgUpdateMeas_,msgTime);
        imgUpdateMeas_.template get<mtImgMeas::_aux>().reset(msgTime);
        updateAndPublish();
      }
    }
  }

  /** \brief Callback for external groundtruth as TransformStamped
   *
   *  @param transform - Groundtruth message.
   */
  void groundtruthCallback(const geometry_msgs::msg::TransformStamped::ConstSharedPtr& transform){
    std::lock_guard<std::mutex> lock(m_filter_);
    if(init_state_.isInitialized()){
      Eigen::Vector3d JrJV(transform->transform.translation.x,transform->transform.translation.y,transform->transform.translation.z);
      poseUpdateMeas_.pos() = JrJV;
      QPD qJV(transform->transform.rotation.w,transform->transform.rotation.x,transform->transform.rotation.y,transform->transform.rotation.z);
      poseUpdateMeas_.att() = qJV.inverted();
      mpFilter_->template addUpdateMeas<1>(poseUpdateMeas_,rclcpp::Time(transform->header.stamp).seconds()+mpPoseUpdate_->timeOffset_);
      updateAndPublish();
    }
  }

  /** \brief Callback for external groundtruth as Odometry
   *
   * @param odometry - Groundtruth message.
   */
  void groundtruthOdometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& odometry) {
    std::lock_guard<std::mutex> lock(m_filter_);
    if(init_state_.isInitialized()) {
      Eigen::Vector3d JrJV(odometry->pose.pose.position.x,odometry->pose.pose.position.y,odometry->pose.pose.position.z);
      poseUpdateMeas_.pos() = JrJV;

      QPD qJV(odometry->pose.pose.orientation.w,odometry->pose.pose.orientation.x,odometry->pose.pose.orientation.y,odometry->pose.pose.orientation.z);
      poseUpdateMeas_.att() = qJV.inverted();

      const Eigen::Matrix<double,6,6> measuredCov = Eigen::Map<const Eigen::Matrix<double,6,6,Eigen::RowMajor>>(odometry->pose.covariance.data());
      poseUpdateMeas_.measuredCov() = measuredCov;

      mpFilter_->template addUpdateMeas<1>(poseUpdateMeas_,rclcpp::Time(odometry->header.stamp).seconds()+mpPoseUpdate_->timeOffset_);
      updateAndPublish();
    }
  }

  /** \brief Callback for external velocity measurements
   *
   *  @param transform - Groundtruth message.
   */
  void velocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& velocity){
    std::lock_guard<std::mutex> lock(m_filter_);
    if(init_state_.isInitialized()){
      Eigen::Vector3d AvM(velocity->twist.linear.x,velocity->twist.linear.y,velocity->twist.linear.z);
      velocityUpdateMeas_.vel() = AvM;
      mpFilter_->template addUpdateMeas<2>(velocityUpdateMeas_,rclcpp::Time(velocity->header.stamp).seconds());
      updateAndPublish();
    }
  }

  /** \brief ROS service handler for resetting the filter.
   */
  void resetServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
                            std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/){
    requestReset();
  }

  /** \brief ROS service handler for resetting the filter to a given pose.
   */
  void resetToPoseServiceCallback(const std::shared_ptr<rovio_interfaces::srv::SrvResetToPose::Request> request,
                                  std::shared_ptr<rovio_interfaces::srv::SrvResetToPose::Response> /*response*/) {
    V3D WrWM(request->t_wm.position.x, request->t_wm.position.y,
             request->t_wm.position.z);
    QPD qWM(request->t_wm.orientation.w, request->t_wm.orientation.x,
            request->t_wm.orientation.y, request->t_wm.orientation.z);
    requestResetToPose(WrWM, qWM.inverted());
  }

  /** \brief Reset the filter when the next IMU measurement is received.
   *         The orientaetion is initialized using an accel. measurement.
   */
  void requestReset() {
    std::lock_guard<std::mutex> lock(m_filter_);
    if (!init_state_.isInitialized()) {
      std::cout << "Reinitialization already triggered. Ignoring request...";
      return;
    }

    init_state_.state_ = FilterInitializationState::State::WaitForInitUsingAccel;
  }

  /** \brief Reset the filter when the next IMU measurement is received.
   *         The pose is initialized to the passed pose.
   *  @param WrWM - Position Vector, pointing from the World-Frame to the IMU-Frame, expressed in World-Coordinates.
   *  @param qMW  - Quaternion, expressing World-Frame in IMU-Coordinates (World Coordinates->IMU Coordinates)
   */
  void requestResetToPose(const V3D& WrWM, const QPD& qMW) {
    std::lock_guard<std::mutex> lock(m_filter_);
    if (!init_state_.isInitialized()) {
      std::cout << "Reinitialization already triggered. Ignoring request...";
      return;
    }

    init_state_.WrWM_ = WrWM;
    init_state_.qMW_ = qMW;
    init_state_.state_ = FilterInitializationState::State::WaitForInitExternalPose;
  }

  /** \brief Executes the update step of the filter and publishes the updated data.
   */
  void updateAndPublish(){
    //
  }
};

}


#endif /* ROVIO_ROVIONODE_HPP_ */
