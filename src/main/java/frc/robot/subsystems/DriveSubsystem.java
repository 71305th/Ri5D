// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  private final CANSparkMax m_motorFrontLeft = new CANSparkMax(DriveConstants.kMotorFrontLeft, MotorType.kBrushless);
  private final CANSparkMax m_motorFrontRight = new CANSparkMax(DriveConstants.kMotorFrontRight, MotorType.kBrushless);
  private final CANSparkMax m_motorRearLeft = new CANSparkMax(DriveConstants.kMotorRearLeft, MotorType.kBrushless);
  private final CANSparkMax m_motorRearRight = new CANSparkMax(DriveConstants.kMotorRearRight, MotorType.kBrushless);

  private final MotorControllerGroup rightGroup = new MotorControllerGroup(m_motorFrontRight, m_motorRearRight);
  private final MotorControllerGroup leftGroup = new MotorControllerGroup(m_motorFrontLeft, m_motorRearLeft);
  private final DifferentialDrive m_drive = new DifferentialDrive(leftGroup, rightGroup);

  private final AHRS m_gyro = new AHRS(Port.kMXP);

  //private final CANCoder leftEncoder = new CANCoder(3);
  //private final CANCoder rightEncoder = new CANCoder(4);

  DifferentialDriveOdometry m_odometry = 
    new DifferentialDriveOdometry(m_gyro.getRotation2d(), getLeftRelativeDistance(), getRightRelativeDistance());

    /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    m_motorFrontLeft.setInverted(true);
    m_motorRearLeft.setInverted(true);
    m_motorFrontRight.setInverted(false);
    m_motorRearRight.setInverted(false);
    
    resetEncoders();
    zeroHeading();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), getLeftRelativeDistance(), getRightRelativeDistance());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      m_odometry.update(m_gyro.getRotation2d(), getLeftRelativeDistance(), getRightRelativeDistance());
      SmartDashboard.putNumber("LeftDis", getLeftRelativeDistance());
      SmartDashboard.putNumber("RightDis", getRightRelativeDistance());
      SmartDashboard.putNumber("Heading", getHeading());
      SmartDashboard.putNumber("PoseX", getPose().getX());
      SmartDashboard.putNumber("PoseY", getPose().getY());
      SmartDashboard.putNumber("LeftVel", getLeftVelocity());
      SmartDashboard.putNumber("RightVel", getRightVelocity());

      //SmartDashboard.putNumber("Left Encoder", leftEncoder.getPosition());
      //SmartDashboard.putNumber("Right Encoder", rightEncoder.getPosition());
    }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void testMotor(){
    this.m_motorRearRight.set(0.1);
  }

  public double getLeftRelativeDistance() {
    double RLencoderCount = m_motorRearLeft.getEncoder().getPosition();
    double FLencoderCount = m_motorFrontLeft.getEncoder().getPosition();
    double averageCount = (RLencoderCount+FLencoderCount)/2;
    return averageCount * DriveConstants.kWheelCircumference * DriveConstants.kGearRatio;
    //* DriveConstants.kDistancePerPulse;
  }

  public double getRightRelativeDistance() {
    double RRencoderCount = m_motorRearRight.getEncoder().getPosition();
    double FRencoderCount = m_motorFrontRight.getEncoder().getPosition();
    double averageCount = (RRencoderCount+FRencoderCount)/2;
    return averageCount * DriveConstants.kWheelCircumference *DriveConstants.kGearRatio;
    // * DriveConstants.kDistancePerPulse;
  }
/*
  public double getleftAbsoluteDistance() {
    return m_leftEncoder.getAbsolutePosition() * DriveConstants.kDistancePerPulse;
  }

  public double getRightAbsoluteDistance() {
    return m_rightEncoder.getAbsolutePosition() * DriveConstants.kDistancePerPulse;
  }
*/
  public double getLeftVelocity() {
    double RLencoderV = m_motorRearLeft.getEncoder().getVelocity();
    double FLencoderV = m_motorFrontLeft.getEncoder().getVelocity();
    double averageV = (RLencoderV+FLencoderV)/2;
    return averageV * DriveConstants.kDistancePerPulse;
  }

  public double getRightVelocity() {
    double RRencoderV = m_motorRearRight.getEncoder().getVelocity();
    double FRencoderV = m_motorFrontRight.getEncoder().getVelocity();
    double averageV = (RRencoderV+FRencoderV)/2;
    return averageV * DriveConstants.kDistancePerPulse;
  }

  public void arcadeDrive(double speed, double rotation) {
    m_drive.arcadeDrive(speed, -rotation);
  }

  public void tankDrive(double left, double right) {
    m_drive.tankDrive(left, right);
  }

  public void setLeftSpeed(double speed) {
    leftGroup.set(speed);
  }

  public void setRightSpeed(double speed) {
    rightGroup.set(speed);
  }
  
  public void setMotor2zero() {
    m_drive.arcadeDrive(0, 0);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d initialPose) {
    resetEncoders();
    m_odometry.resetPosition(
      m_gyro.getRotation2d(), getLeftRelativeDistance(), getRightRelativeDistance(), initialPose);
  }

  public void resetEncoders() {
    m_motorRearLeft.getEncoder().setPosition(0.0);
    m_motorFrontLeft.getEncoder().setPosition(0.0);
    m_motorRearRight.getEncoder().setPosition(0.0);
    m_motorFrontRight.getEncoder().setPosition(0.0);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftGroup.setVoltage(leftVolts);
    rightGroup.setVoltage(rightVolts);
    m_drive.feed();
  }

  public double getAverageEncoderRelativeDistance() {
    return (getLeftRelativeDistance() + getRightRelativeDistance()) / 2.0;
  }
  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
    m_gyro.zeroYaw();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getYaw();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}