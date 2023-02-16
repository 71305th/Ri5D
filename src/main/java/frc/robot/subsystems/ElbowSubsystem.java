// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElbowConstants;

public class ElbowSubsystem extends SubsystemBase {

  private final CANSparkMax m_elbowMotor = new CANSparkMax(ElbowConstants.motorID, MotorType.kBrushless);
  // private final CANcoder m_elbowCancoder = new CANcoder(ElbowConstants.cancoderID);
  
  /** Creates a new LufySubsystem. */
  public ElbowSubsystem() {
    m_elbowMotor.setInverted(false);
    m_elbowMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void elbowRun(double speed){
    m_elbowMotor.set(speed);
  }
  
  public void elbowStop(){
    m_elbowMotor.set(0);
  }

  public double getArmAngPos() {
    return m_elbowMotor.getEncoder().getPosition() * 2 * Math.PI / ElbowConstants.kEncoderCPR / ElbowConstants.kGearRatio;
  }

  public double getArmAngVel() {
    return m_elbowMotor.getEncoder().getVelocity() * 2 * Math.PI / ElbowConstants.kEncoderCPR / ElbowConstants.kGearRatio;
  }

  public void resetEncoders() {
   m_elbowMotor.getEncoder().setPosition(0);
  }

  
}
