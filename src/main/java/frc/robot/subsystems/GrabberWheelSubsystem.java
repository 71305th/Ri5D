// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;

public class GrabberWheelSubsystem extends SubsystemBase {

  private final CANSparkMax m_grabberMotor = new CANSparkMax(GrabberConstants.motorID, MotorType.kBrushless);

  /** Creates a new GrabberWheelSubsystem. */
  public GrabberWheelSubsystem() {
    m_grabberMotor.setInverted(false);
    m_grabberMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void rollRun(double speed){
    m_grabberMotor.set(speed);
  }

  public void rollStop(){
    m_grabberMotor.set(0);
  }

  public double getWheelPos(){
   return m_grabberMotor.getEncoder().getPosition();
  }

  public double getWheelVel(){
    return m_grabberMotor.getEncoder().getVelocity();
  }

  public void resetEncoders(){
    m_grabberMotor.getEncoder().setPosition(0);
  }
}
