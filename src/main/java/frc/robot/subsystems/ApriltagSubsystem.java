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
import edu.wpi.first.math.geometry.Translation3d;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ApriltagSubsystem extends SubsystemBase {
  
  PhotonCamera pv_cam = new PhotonCamera("WEB_CAM");
  PhotonPipelineResult result;
  boolean hasTarget;
  List<PhotonTrackedTarget> targets;

  //define target
  PhotonTrackedTarget target;

  //define information of the target

  double yaw;
  double pitch;
  double area;
  double skew;
  int targetID;
  double poseAmbiguity;

  Transform3d bestCameraToTarget;
  Transform3d alternateCameraToTarget;
  List<TargetCorner> corners;

  public ApriltagSubsystem() {}

  @Override
  public void periodic() {
    result = pv_cam.getLatestResult();
    hasTarget = result.hasTargets();

    if (hasTarget){
      target = result.getBestTarget();
      yaw = target.getYaw();
      pitch = target.getPitch();
      area = target.getArea();
      skew = target.getSkew();
      targetID = target.getFiducialId();
      poseAmbiguity = target.getPoseAmbiguity();

      bestCameraToTarget = target.getBestCameraToTarget();
      corners = target.getDetectedCorners();

      getCameratoTarget();
    }

    // SmartDashboard.putNumber("targetID", targetID);
    // SmartDashboard.putNumber("yaw", yaw);
    // SmartDashboard.putNumber("skew", skew);
    // SmartDashboard.putNumber("area", area); 
    // SmartDashboard.putNumber("pitch", pitch);
    // SmartDashboard.putNumber("poseAmbiguity", poseAmbiguity);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void snapshot(){
    pv_cam.takeInputSnapshot();
    pv_cam.takeOutputSnapshot();
  }

  /**
   * @return
   * x -> front</p>
   * y -> left</p>
   * z -> up</p>
   **/
  public Transform3d getCameratoTarget(){
    return hasTarget == true ? target.getBestCameraToTarget() : new Transform3d( new Translation3d(0,0,0), new Rotation3d(0,0,0) );
  }


  public int getTargetID(){
    return hasTarget == true ? target.getFiducialId() : 0;
  }

  /**
   * @return degrees
   **/
  public double getYaw(){
    return hasTarget == true ? target.getYaw() : 0;
  }

  /**
   * @return degrees
   **/
  public double getSkew(){
    return hasTarget == true ? target.getSkew() : 0;
  }

  /**
   * @return degrees
   **/
  public double getPitch(){
    return hasTarget == true ? target.getPitch() : 0;
  }

  public boolean hasTarget(){
    return hasTarget;    
  }

  public double getPoseAmbiguity(){
    return hasTarget == true ? target.getPoseAmbiguity() : 0;
  }
}