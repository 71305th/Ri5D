// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ApriltagConstants;
import frc.robot.Constants.Field;

public class ApriltagSubsystem extends SubsystemBase {
  
  PhotonCamera mPVCamera = new PhotonCamera("WEB_CAM");
  PhotonPipelineResult mResult;
  boolean mHasTarget;
  List<PhotonTrackedTarget> mTargets;

  //define target
  PhotonTrackedTarget mTarget;

  //define information of the target
  double mYaw;
  double mPitch;
  double mArea;
  double mSkew;
  double mPoseAmbiguity;

  Transform3d mBestCameraToTarget;
  Transform3d mAlternateCameraToTarget;
  List<TargetCorner> mCorners;

  /**
   * Define the position vaeiable in meters
   */
  double mx, my;
  int mTargetID;
  Translation2d mApriltagIDPosition;
  Transform3d mError;

  public ApriltagSubsystem() {
    mTargetID = 0;
  }

  @Override
  public void periodic() {
    variableSettoZero();
    mResult = mPVCamera.getLatestResult();
    mHasTarget = mResult.hasTargets();

    if( mHasTarget ){
      mTarget = mResult.getBestTarget();
      mYaw = mTarget.getYaw();
      mPitch = mTarget.getPitch();
      mArea = mTarget.getArea();
      mSkew = mTarget.getSkew();
      mTargetID = mTarget.getFiducialId();
      mPoseAmbiguity = mTarget.getPoseAmbiguity();

      mBestCameraToTarget = mTarget.getBestCameraToTarget();
      mCorners = mTarget.getDetectedCorners();

      getCameratoTarget();
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void variableSettoZero(){
    mYaw = 0;
    mPitch = 0;
    mArea = 0;
    mSkew = 0;
    mTargetID = 0;
    mPoseAmbiguity = 0;
  }

  public void snapshot(){
    mPVCamera.takeInputSnapshot();
    mPVCamera.takeOutputSnapshot();
  }

  /**
   * @return Your Position On Field. If there isn't any target return (-1.0, -1.0)
   */
  public Translation2d getPosByApriltag(){
    mTargetID = getTargetID();
    Translation2d mApriltagPosition = new Translation2d(-1.0, -1.0);

    if( mTargetID != 0 ){
        mApriltagIDPosition = Field.fieldmMap.get(mTargetID);
        mError = getCameratoTarget();

        mApriltagPosition = new Translation2d(
          ApriltagConstants.kX * (mApriltagIDPosition.getX() + mError.getX()),
          ApriltagConstants.kY * (mApriltagIDPosition.getY() + mError.getY())
        );
    }

    return mApriltagPosition;
  }

  /**
   * @return
   * x -> front</p>
   * y -> left</p>
   * z -> up</p>
   **/
  public Transform3d getCameratoTarget(){
    return hasTarget() == true ? mTarget.getBestCameraToTarget() : new Transform3d( new Translation3d(0,0,0), new Rotation3d(0,0,0) );
  }


  public int getTargetID(){
    return hasTarget() == true ? mTarget.getFiducialId() : 0;
  }

  /**
   * @return degrees
   **/
  public double getYaw(){
    return hasTarget() == true ? mTarget.getYaw() : 0;
  }

  /**
   * @return degrees
   **/
  public double getSkew(){
    return hasTarget() == true ? mTarget.getSkew() : 0;
  }

  /**
   * @return degrees
   **/
  public double getPitch(){
    return hasTarget() == true ? mTarget.getPitch() : 0;
  }

  public boolean hasTarget(){
    return hasTarget();
  }

  public double getPoseAmbiguity(){
    return hasTarget() == true ? mTarget.getPoseAmbiguity() : 0;
  }
}
