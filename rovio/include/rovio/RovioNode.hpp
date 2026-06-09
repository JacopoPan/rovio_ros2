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
    //
  }

  /** \brief Destructor
   */
  virtual ~RovioNode(){}

  /** \brief Tests the functionality of the rovio node.
   *
   *  @todo debug with   doVECalibration = false and depthType = 0
   */
  void makeTest(){
    //
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
