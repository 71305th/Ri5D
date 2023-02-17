// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.ApriltagSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ApriltagCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  // Declare the subsystems
  private DriveSubsystem drive;
  private ApriltagSubsystem apriltag;
  private final int targetID;

  public ApriltagCommand(ApriltagSubsystem apriltag, DriveSubsystem drive, int targetID) {
    this.apriltag = apriltag;
    this.drive = drive;
    this.targetID = targetID;
    
    addRequirements(apriltag, drive);
  }

  // Variables
  double mForward;
  double mTurn;
  double mTime;  
  double mLastTime;
  double mDeltaT;
  double mDeltaForward;
  double mDeltaTurn;
  double mSum_Forward;
  double mSum_Turn;
  double mOutputForward;
  double mOutputTurn;

  // Normal Target
  boolean mHasTarget;
  Transform3d mTargetToCamera;

  // Position
  Translation2d mPositionOnFieldByApriltag;

  @Override
  public void initialize() {
    drive.resetEncoders();
  }

  @Override
  public void execute() {
    mHasTarget = apriltag.hasTarget();
    mTargetToCamera = apriltag.getCameratoTarget();
    mPositionOnFieldByApriltag = apriltag.getPosByApriltag();
    
    // mForward = 0;
    // mTurn = 0;
    // if( apriltag.getTargetID() == targetID ){
    //   mForward = mTargetToCamera.getX() - 0.36;
    //   mTurn = -mTargetToCamera.getY();
    //   mTime = Timer.getFPGATimestamp();
  
    //   mDeltaT = mLastTime - mTime;
    //   mDeltaForward = mForward / mDeltaT;
    //   mDeltaTurn = mTurn / mDeltaT;
  
    //   if( mForward < 1 ) mSum_Forward += mForward;
    //   if( mTurn < 0.3 ) mSum_Turn += mTurn;
  
    //   mOutputForward = PIDConstants.kP_foward * mForward + PIDConstants.kI_foward * mSum_Forward + PIDConstants.kD_foward * mDeltaForward;
    //   mOutputTurn = -1 * (PIDConstants.kP_turn * mTurn + PIDConstants.kI_turn * mSum_Turn + PIDConstants.kD_turn * mDeltaTurn);
  
    //   drive.arcadeDrive(mOutputForward, mOutputTurn);
    // }else drive.arcadeDrive(0, 0.3);

    // mLastTime = mTime;
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}