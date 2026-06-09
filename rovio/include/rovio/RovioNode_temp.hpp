  /** \brief Executes the update step of the filter and publishes the updated data.
   */
  void updateAndPublish(){
    if(init_state_.isInitialized()){
      // Execute the filter update.
      const double t1 = (double) cv::getTickCount();
      static double timing_T = 0;
      static int timing_C = 0;
      const double oldSafeTime = mpFilter_->safe_.t_;
      int c1 = std::get<0>(mpFilter_->updateTimelineTuple_).measMap_.size();
      double lastImageTime;
      if(std::get<0>(mpFilter_->updateTimelineTuple_).getLastTime(lastImageTime)){
        mpFilter_->updateSafe(&lastImageTime);
      }
      const double t2 = (double) cv::getTickCount();
      int c2 = std::get<0>(mpFilter_->updateTimelineTuple_).measMap_.size();
      timing_T += (t2-t1)/cv::getTickFrequency()*1000;
      timing_C += c1-c2;
      bool plotTiming = false;
      if(plotTiming){
        ROS_INFO_STREAM(" == Filter Update: " << (t2-t1)/cv::getTickFrequency()*1000 << " ms for processing " << c1-c2 << " images, average: " << timing_T/timing_C);
      }
      if(mpFilter_->safe_.t_ > oldSafeTime){ // Publish only if something changed
        for(int i=0;i<mtState::nCam_;i++){
          if(!mpFilter_->safe_.img_[i].empty() && mpImgUpdate_->doFrameVisualisation_){
            cv::imshow("Tracker" + std::to_string(i), mpFilter_->safe_.img_[i]);
            cv::waitKey(3);
          }
        }
        if(!mpFilter_->safe_.patchDrawing_.empty() && mpImgUpdate_->visualizePatches_){
          cv::imshow("Patches", mpFilter_->safe_.patchDrawing_);
          cv::waitKey(3);
        }

        // Obtain the save filter state.
        mtFilterState& filterState = mpFilter_->safe_;
	      mtState& state = mpFilter_->safe_.state_;
        state.updateMultiCameraExtrinsics(&mpFilter_->multiCamera_);
        MXD& cov = mpFilter_->safe_.cov_;
        imuOutputCT_.transformState(state,imuOutput_);

        // Cout verbose for pose measurements
        if(mpImgUpdate_->verbose_){
          if(mpPoseUpdate_->inertialPoseIndex_ >=0){
            std::cout << "Transformation between inertial frames, IrIW, qWI: " << std::endl;
            std::cout << "  " << state.poseLin(mpPoseUpdate_->inertialPoseIndex_).transpose() << std::endl;
            std::cout << "  " << state.poseRot(mpPoseUpdate_->inertialPoseIndex_) << std::endl;
          }
          if(mpPoseUpdate_->bodyPoseIndex_ >=0){
            std::cout << "Transformation between body frames, MrMV, qVM: " << std::endl;
            std::cout << "  " << state.poseLin(mpPoseUpdate_->bodyPoseIndex_).transpose() << std::endl;
            std::cout << "  " << state.poseRot(mpPoseUpdate_->bodyPoseIndex_) << std::endl;
          }
        }

        // Send Map (Pose Sensor, I) to World (rovio-intern, W) transformation
        if(mpPoseUpdate_->inertialPoseIndex_ >=0){
          Eigen::Vector3d IrIW = state.poseLin(mpPoseUpdate_->inertialPoseIndex_);
          QPD qWI = state.poseRot(mpPoseUpdate_->inertialPoseIndex_);
          tf::StampedTransform tf_transform_WI;
          tf_transform_WI.frame_id_ = map_frame_;
          tf_transform_WI.child_frame_id_ = world_frame_;
          tf_transform_WI.stamp_ = ros::Time(mpFilter_->safe_.t_);
          tf_transform_WI.setOrigin(tf::Vector3(IrIW(0),IrIW(1),IrIW(2)));
          tf_transform_WI.setRotation(tf::Quaternion(qWI.x(),qWI.y(),qWI.z(),-qWI.w()));
          tb_.sendTransform(tf_transform_WI);
        }

        // Send IMU pose.
        tf::StampedTransform tf_transform_MW;
        tf_transform_MW.frame_id_ = world_frame_;
        tf_transform_MW.child_frame_id_ = imu_frame_;
        tf_transform_MW.stamp_ = ros::Time(mpFilter_->safe_.t_);
        tf_transform_MW.setOrigin(tf::Vector3(imuOutput_.WrWB()(0),imuOutput_.WrWB()(1),imuOutput_.WrWB()(2)));
        tf_transform_MW.setRotation(tf::Quaternion(imuOutput_.qBW().x(),imuOutput_.qBW().y(),imuOutput_.qBW().z(),-imuOutput_.qBW().w()));
        tb_.sendTransform(tf_transform_MW);

        // Send camera pose.
        for(int camID=0;camID<mtState::nCam_;camID++){
          tf::StampedTransform tf_transform_CM;
          tf_transform_CM.frame_id_ = imu_frame_;
          tf_transform_CM.child_frame_id_ = camera_frame_ + std::to_string(camID);
          tf_transform_CM.stamp_ = ros::Time(mpFilter_->safe_.t_);
          tf_transform_CM.setOrigin(tf::Vector3(state.MrMC(camID)(0),state.MrMC(camID)(1),state.MrMC(camID)(2)));
          tf_transform_CM.setRotation(tf::Quaternion(state.qCM(camID).x(),state.qCM(camID).y(),state.qCM(camID).z(),-state.qCM(camID).w()));
          tb_.sendTransform(tf_transform_CM);
        }

        // Publish Odometry
        if(pubOdometry_.getNumSubscribers() > 0 || forceOdometryPublishing_){
          // Compute covariance of output
          imuOutputCT_.transformCovMat(state,cov,imuOutputCov_);

          odometryMsg_.header.seq = msgSeq_;
          odometryMsg_.header.stamp = ros::Time(mpFilter_->safe_.t_);
          odometryMsg_.pose.pose.position.x = imuOutput_.WrWB()(0);
          odometryMsg_.pose.pose.position.y = imuOutput_.WrWB()(1);
          odometryMsg_.pose.pose.position.z = imuOutput_.WrWB()(2);
          odometryMsg_.pose.pose.orientation.w = -imuOutput_.qBW().w();
          odometryMsg_.pose.pose.orientation.x = imuOutput_.qBW().x();
          odometryMsg_.pose.pose.orientation.y = imuOutput_.qBW().y();
          odometryMsg_.pose.pose.orientation.z = imuOutput_.qBW().z();
          for(unsigned int i=0;i<6;i++){
            unsigned int ind1 = mtOutput::template getId<mtOutput::_pos>()+i;
            if(i>=3) ind1 = mtOutput::template getId<mtOutput::_att>()+i-3;
            for(unsigned int j=0;j<6;j++){
              unsigned int ind2 = mtOutput::template getId<mtOutput::_pos>()+j;
              if(j>=3) ind2 = mtOutput::template getId<mtOutput::_att>()+j-3;
              odometryMsg_.pose.covariance[j+6*i] = imuOutputCov_(ind1,ind2);
            }
          }
          odometryMsg_.twist.twist.linear.x = imuOutput_.BvB()(0);
          odometryMsg_.twist.twist.linear.y = imuOutput_.BvB()(1);
          odometryMsg_.twist.twist.linear.z = imuOutput_.BvB()(2);
          odometryMsg_.twist.twist.angular.x = imuOutput_.BwWB()(0);
          odometryMsg_.twist.twist.angular.y = imuOutput_.BwWB()(1);
          odometryMsg_.twist.twist.angular.z = imuOutput_.BwWB()(2);
          for(unsigned int i=0;i<6;i++){
            unsigned int ind1 = mtOutput::template getId<mtOutput::_vel>()+i;
            if(i>=3) ind1 = mtOutput::template getId<mtOutput::_ror>()+i-3;
            for(unsigned int j=0;j<6;j++){
              unsigned int ind2 = mtOutput::template getId<mtOutput::_vel>()+j;
              if(j>=3) ind2 = mtOutput::template getId<mtOutput::_ror>()+j-3;
              odometryMsg_.twist.covariance[j+6*i] = imuOutputCov_(ind1,ind2);
            }
          }
          pubOdometry_.publish(odometryMsg_);
        }

        if(pubPoseWithCovStamped_.getNumSubscribers() > 0 || forcePoseWithCovariancePublishing_){
          // Compute covariance of output
          imuOutputCT_.transformCovMat(state,cov,imuOutputCov_);

          estimatedPoseWithCovarianceStampedMsg_.header.seq = msgSeq_;
          estimatedPoseWithCovarianceStampedMsg_.header.stamp = ros::Time(mpFilter_->safe_.t_);
          estimatedPoseWithCovarianceStampedMsg_.pose.pose.position.x = imuOutput_.WrWB()(0);
          estimatedPoseWithCovarianceStampedMsg_.pose.pose.position.y = imuOutput_.WrWB()(1);
          estimatedPoseWithCovarianceStampedMsg_.pose.pose.position.z = imuOutput_.WrWB()(2);
          estimatedPoseWithCovarianceStampedMsg_.pose.pose.orientation.w = -imuOutput_.qBW().w();
          estimatedPoseWithCovarianceStampedMsg_.pose.pose.orientation.x = imuOutput_.qBW().x();
          estimatedPoseWithCovarianceStampedMsg_.pose.pose.orientation.y = imuOutput_.qBW().y();
          estimatedPoseWithCovarianceStampedMsg_.pose.pose.orientation.z = imuOutput_.qBW().z();

          for(unsigned int i=0;i<6;i++){
            unsigned int ind1 = mtOutput::template getId<mtOutput::_pos>()+i;
            if(i>=3) ind1 = mtOutput::template getId<mtOutput::_att>()+i-3;
            for(unsigned int j=0;j<6;j++){
              unsigned int ind2 = mtOutput::template getId<mtOutput::_pos>()+j;
              if(j>=3) ind2 = mtOutput::template getId<mtOutput::_att>()+j-3;
              estimatedPoseWithCovarianceStampedMsg_.pose.covariance[j+6*i] = imuOutputCov_(ind1,ind2);
            }
          }

          pubPoseWithCovStamped_.publish(estimatedPoseWithCovarianceStampedMsg_);

        }

        // Send IMU pose message.
        if(pubTransform_.getNumSubscribers() > 0 || forceTransformPublishing_){
          transformMsg_.header.seq = msgSeq_;
          transformMsg_.header.stamp = ros::Time(mpFilter_->safe_.t_);
          transformMsg_.transform.translation.x = imuOutput_.WrWB()(0);
          transformMsg_.transform.translation.y = imuOutput_.WrWB()(1);
          transformMsg_.transform.translation.z = imuOutput_.WrWB()(2);
          transformMsg_.transform.rotation.x = imuOutput_.qBW().x();
          transformMsg_.transform.rotation.y = imuOutput_.qBW().y();
          transformMsg_.transform.rotation.z = imuOutput_.qBW().z();
          transformMsg_.transform.rotation.w = -imuOutput_.qBW().w();
          pubTransform_.publish(transformMsg_);
        }

        if(pub_T_J_W_transform.getNumSubscribers() > 0 || forceTransformPublishing_){
          if (mpPoseUpdate_->inertialPoseIndex_ >= 0) {
            Eigen::Vector3d IrIW = state.poseLin(mpPoseUpdate_->inertialPoseIndex_);
            QPD qWI = state.poseRot(mpPoseUpdate_->inertialPoseIndex_);
            T_J_W_Msg_.header.seq = msgSeq_;
            T_J_W_Msg_.header.stamp = ros::Time(mpFilter_->safe_.t_);
            T_J_W_Msg_.transform.translation.x = IrIW(0);
            T_J_W_Msg_.transform.translation.y = IrIW(1);
            T_J_W_Msg_.transform.translation.z = IrIW(2);
            T_J_W_Msg_.transform.rotation.x = qWI.x();
            T_J_W_Msg_.transform.rotation.y = qWI.y();
            T_J_W_Msg_.transform.rotation.z = qWI.z();
            T_J_W_Msg_.transform.rotation.w = -qWI.w();
            pub_T_J_W_transform.publish(T_J_W_Msg_);
          }
        }

        // Publish Extrinsics
        for(int camID=0;camID<mtState::nCam_;camID++){
          if(pubExtrinsics_[camID].getNumSubscribers() > 0 || forceExtrinsicsPublishing_){
            extrinsicsMsg_[camID].header.seq = msgSeq_;
            extrinsicsMsg_[camID].header.stamp = ros::Time(mpFilter_->safe_.t_);
            extrinsicsMsg_[camID].pose.pose.position.x = state.MrMC(camID)(0);
            extrinsicsMsg_[camID].pose.pose.position.y = state.MrMC(camID)(1);
            extrinsicsMsg_[camID].pose.pose.position.z = state.MrMC(camID)(2);
            extrinsicsMsg_[camID].pose.pose.orientation.x = state.qCM(camID).x();
            extrinsicsMsg_[camID].pose.pose.orientation.y = state.qCM(camID).y();
            extrinsicsMsg_[camID].pose.pose.orientation.z = state.qCM(camID).z();
            extrinsicsMsg_[camID].pose.pose.orientation.w = -state.qCM(camID).w();
            for(unsigned int i=0;i<6;i++){
              unsigned int ind1 = mtState::template getId<mtState::_vep>(camID)+i;
              if(i>=3) ind1 = mtState::template getId<mtState::_vea>(camID)+i-3;
              for(unsigned int j=0;j<6;j++){
                unsigned int ind2 = mtState::template getId<mtState::_vep>(camID)+j;
                if(j>=3) ind2 = mtState::template getId<mtState::_vea>(camID)+j-3;
                extrinsicsMsg_[camID].pose.covariance[j+6*i] = cov(ind1,ind2);
              }
            }
            pubExtrinsics_[camID].publish(extrinsicsMsg_[camID]);
          }
        }

        // Publish IMU biases
        if(pubImuBias_.getNumSubscribers() > 0 || forceImuBiasPublishing_){
          imuBiasMsg_.header.seq = msgSeq_;
          imuBiasMsg_.header.stamp = ros::Time(mpFilter_->safe_.t_);
          imuBiasMsg_.angular_velocity.x = state.gyb()(0);
          imuBiasMsg_.angular_velocity.y = state.gyb()(1);
          imuBiasMsg_.angular_velocity.z = state.gyb()(2);
          imuBiasMsg_.linear_acceleration.x = state.acb()(0);
          imuBiasMsg_.linear_acceleration.y = state.acb()(1);
          imuBiasMsg_.linear_acceleration.z = state.acb()(2);
          for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
              imuBiasMsg_.angular_velocity_covariance[3*i+j] = cov(mtState::template getId<mtState::_gyb>()+i,mtState::template getId<mtState::_gyb>()+j);
            }
          }
          for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
              imuBiasMsg_.linear_acceleration_covariance[3*i+j] = cov(mtState::template getId<mtState::_acb>()+i,mtState::template getId<mtState::_acb>()+j);
            }
          }
          pubImuBias_.publish(imuBiasMsg_);
        }

        // PointCloud message.
        if(pubPcl_.getNumSubscribers() > 0 || pubMarkers_.getNumSubscribers() > 0 || forcePclPublishing_ || forceMarkersPublishing_){
          pclMsg_.header.seq = msgSeq_;
          pclMsg_.header.stamp = ros::Time(mpFilter_->safe_.t_);
          markerMsg_.header.seq = msgSeq_;
          markerMsg_.header.stamp = ros::Time(mpFilter_->safe_.t_);
          markerMsg_.points.clear();
          float badPoint = std::numeric_limits<float>::quiet_NaN();  // Invalid point.
          int offset = 0;

          FeatureDistance distance;
          double d,d_minus,d_plus;
          const double stretchFactor = 3;
          for (unsigned int i=0;i<mtState::nMax_; i++, offset += pclMsg_.point_step) {
            if(filterState.fsm_.isValid_[i]){
              // Get 3D feature coordinates.
              int camID = filterState.fsm_.features_[i].mpCoordinates_->camID_;
              distance = state.dep(i);
              d = distance.getDistance();
              const double sigma = sqrt(cov(mtState::template getId<mtState::_fea>(i)+2,mtState::template getId<mtState::_fea>(i)+2));
              distance.p_ -= stretchFactor*sigma;
              d_minus = distance.getDistance();
              if(d_minus > 1000) d_minus = 1000;
              if(d_minus < 0) d_minus = 0;
              distance.p_ += 2*stretchFactor*sigma;
              d_plus = distance.getDistance();
              if(d_plus > 1000) d_plus = 1000;
              if(d_plus < 0) d_plus = 0;
              Eigen::Vector3d bearingVector = filterState.state_.CfP(i).get_nor().getVec();
              const Eigen::Vector3d CrCPm = bearingVector*d_minus;
              const Eigen::Vector3d CrCPp = bearingVector*d_plus;
              const Eigen::Vector3f MrMPm = V3D(mpFilter_->multiCamera_.BrBC_[camID] + mpFilter_->multiCamera_.qCB_[camID].inverseRotate(CrCPm)).cast<float>();
              const Eigen::Vector3f MrMPp = V3D(mpFilter_->multiCamera_.BrBC_[camID] + mpFilter_->multiCamera_.qCB_[camID].inverseRotate(CrCPp)).cast<float>();

              // Get human readable output
              transformFeatureOutputCT_.setFeatureID(i);
              transformFeatureOutputCT_.setOutputCameraID(filterState.fsm_.features_[i].mpCoordinates_->camID_);
              transformFeatureOutputCT_.transformState(state,featureOutput_);
              transformFeatureOutputCT_.transformCovMat(state,cov,featureOutputCov_);
              featureOutputReadableCT_.transformState(featureOutput_,featureOutputReadable_);
              featureOutputReadableCT_.transformCovMat(featureOutput_,featureOutputCov_,featureOutputReadableCov_);

              // Get landmark output
              landmarkOutputImuCT_.setFeatureID(i);
              landmarkOutputImuCT_.transformState(state,landmarkOutput_);
              landmarkOutputImuCT_.transformCovMat(state,cov,landmarkOutputCov_);
              const Eigen::Vector3f MrMP = landmarkOutput_.get<LandmarkOutput::_lmk>().template cast<float>();

              // Write feature id, camera id, and rgb
              uint8_t gray = 255;
              uint32_t rgb = (gray << 16) | (gray << 8) | gray;
              uint32_t status = filterState.fsm_.features_[i].mpStatistics_->status_[0];
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[0].offset], &filterState.fsm_.features_[i].idx_, sizeof(int));  // id
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[1].offset], &camID, sizeof(int));  // cam id
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[2].offset], &rgb, sizeof(uint32_t));  // rgb
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[3].offset], &status, sizeof(int));  // status

              // Write coordinates to pcl message.
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[4].offset], &MrMP[0], sizeof(float));  // x
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[5].offset], &MrMP[1], sizeof(float));  // y
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[6].offset], &MrMP[2], sizeof(float));  // z

              // Add feature bearing vector and distance
              const Eigen::Vector3f bearing = featureOutputReadable_.bea().template cast<float>();
              const float distance = static_cast<float>(featureOutputReadable_.dis());
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[7].offset], &bearing[0], sizeof(float));  // x
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[8].offset], &bearing[1], sizeof(float));  // y
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[9].offset], &bearing[2], sizeof(float));  // z
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[10].offset], &distance, sizeof(float)); // d

              // Add the corresponding covariance (upper triangular)
              Eigen::Matrix3f cov_MrMP = landmarkOutputCov_.cast<float>();
              int mCounter = 11;
              for(int row=0;row<3;row++){
                for(int col=row;col<3;col++){
                  memcpy(&pclMsg_.data[offset + pclMsg_.fields[mCounter].offset], &cov_MrMP(row,col), sizeof(float));
                  mCounter++;
                }
              }

              // Add distance uncertainty
              const float distance_cov = static_cast<float>(featureOutputReadableCov_(3,3));
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[mCounter].offset], &distance_cov, sizeof(float));

              // Line markers (Uncertainty rays).
              geometry_msgs::Point point_near_msg;
              geometry_msgs::Point point_far_msg;
              point_near_msg.x = float(CrCPp[0]);
              point_near_msg.y = float(CrCPp[1]);
              point_near_msg.z = float(CrCPp[2]);
              point_far_msg.x = float(CrCPm[0]);
              point_far_msg.y = float(CrCPm[1]);
              point_far_msg.z = float(CrCPm[2]);
              markerMsg_.points.push_back(point_near_msg);
              markerMsg_.points.push_back(point_far_msg);
            }
            else {
              // If current feature is not valid copy NaN
              int id = -1;
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[0].offset], &id, sizeof(int));  // id
              for(int j=1;j<pclMsg_.fields.size();j++){
                memcpy(&pclMsg_.data[offset + pclMsg_.fields[j].offset], &badPoint, sizeof(float));
              }
            }
          }
          pubPcl_.publish(pclMsg_);
          pubMarkers_.publish(markerMsg_);
        }
        if(pubPatch_.getNumSubscribers() > 0 || forcePatchPublishing_){
          patchMsg_.header.seq = msgSeq_;
          patchMsg_.header.stamp = ros::Time(mpFilter_->safe_.t_);
          int offset = 0;
          for (unsigned int i=0;i<mtState::nMax_; i++, offset += patchMsg_.point_step) {
            if(filterState.fsm_.isValid_[i]){
              memcpy(&patchMsg_.data[offset + patchMsg_.fields[0].offset], &filterState.fsm_.features_[i].idx_, sizeof(int));  // id
              // Add patch data
              for(int l=0;l<mtState::nLevels_;l++){
                for(int y=0;y<mtState::patchSize_;y++){
                  for(int x=0;x<mtState::patchSize_;x++){
                    memcpy(&patchMsg_.data[offset + patchMsg_.fields[1].offset + (l*mtState::patchSize_*mtState::patchSize_ + y*mtState::patchSize_ + x)*4], &filterState.fsm_.features_[i].mpMultilevelPatch_->patches_[l].patch_[y*mtState::patchSize_ + x], sizeof(float)); // Patch
                    memcpy(&patchMsg_.data[offset + patchMsg_.fields[2].offset + (l*mtState::patchSize_*mtState::patchSize_ + y*mtState::patchSize_ + x)*4], &filterState.fsm_.features_[i].mpMultilevelPatch_->patches_[l].dx_[y*mtState::patchSize_ + x], sizeof(float)); // dx
                    memcpy(&patchMsg_.data[offset + patchMsg_.fields[3].offset + (l*mtState::patchSize_*mtState::patchSize_ + y*mtState::patchSize_ + x)*4], &filterState.fsm_.features_[i].mpMultilevelPatch_->patches_[l].dy_[y*mtState::patchSize_ + x], sizeof(float)); // dy
                    memcpy(&patchMsg_.data[offset + patchMsg_.fields[4].offset + (l*mtState::patchSize_*mtState::patchSize_ + y*mtState::patchSize_ + x)*4], &filterState.mlpErrorLog_[i].patches_[l].patch_[y*mtState::patchSize_ + x], sizeof(float)); // error
                  }
                }
              }
            }
            else {
              // If current feature is not valid copy NaN
              int id = -1;
              memcpy(&patchMsg_.data[offset + patchMsg_.fields[0].offset], &id, sizeof(int));  // id
            }
          }

          pubPatch_.publish(patchMsg_);
        }
        gotFirstMessages_ = true;
      }
    }
  }
