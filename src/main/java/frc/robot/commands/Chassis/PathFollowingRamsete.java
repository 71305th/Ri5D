// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class PathFollowingRamsete extends CommandBase {

  private final DriveSubsystem m_drive;

  String pathname;
  boolean isFirstpath;

  RamseteCommand ramseteCommand;
  PPRamseteCommand ppRamseteCommand;
  
  /** Creates a new PathFollowing. */
  public PathFollowingRamsete(DriveSubsystem m_drive, String pathname, boolean isFirstpath) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pathname = pathname;
    this.m_drive = m_drive;
    this.isFirstpath = isFirstpath;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    // var autoVoltageConstraint =
    //     new DifferentialDriveVoltageConstraint(
    //         new SimpleMotorFeedforward(
    //             DriveConstants.ksVolts,
    //             DriveConstants.kvVoltSecondsPerMeter,
    //             DriveConstants.kaVoltSecondsSquaredPerMeter),
    //         DriveConstants.kDriveKinematics,
    //         10);

    // Create config for trajectory
    // TrajectoryConfig config =
    //     new TrajectoryConfig(
    //             AutoConstants.kMaxSpeedMetersPerSecond,
    //             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //         // Add kinematics to ensure max speed is actually obeyed
    //         .setKinematics(DriveConstants.kDriveKinematics)
    //         // Apply the voltage constraint
    //         .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    PathPlannerTrajectory path = 
      PathPlanner.loadPath(
        pathname, new PathConstraints(
          AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));

    // Wpilib Original RamseteCmd
    // ramseteCommand =
    //     new RamseteCommand(
    //         path,
    //         m_drive::getPose,
    //         new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    //         new SimpleMotorFeedforward(
    //             DriveConstants.ksVolts,
    //             DriveConstants.kvVoltSecondsPerMeter,
    //             DriveConstants.kaVoltSecondsSquaredPerMeter),
    //         DriveConstants.kDriveKinematics,
    //         m_drive::getWheelSpeeds,
    //         new PIDController(DriveConstants.kPDriveVel, 0, 0),
    //         new PIDController(DriveConstants.kPDriveVel, 0, 0),
    //         // RamseteCommand passes volts to the callback
    //         m_drive::tankDriveVolts,
    //         m_drive);

    // PathPlanner's RamseteCmd
    ppRamseteCommand = 
      new PPRamseteCommand(
        path, 
        m_drive::getPose, // Pose supplier
        new RamseteController(),
        new SimpleMotorFeedforward(
          DriveConstants.ksVolts, 
          DriveConstants.kvVoltSecondsPerMeter, 
          DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, // DifferentialDriveKinematics
        m_drive::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
        new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
        m_drive::tankDriveVolts, // Voltage biconsumer
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        m_drive // Requires this drive subsystem
      );

    // Reset odometry to the starting pose of the trajectory.
    if (isFirstpath) {
      m_drive.resetOdometry(path.getInitialPose());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // ramseteCommand.execute();
    ppRamseteCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}