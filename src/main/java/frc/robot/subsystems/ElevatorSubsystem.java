// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  private final CANSparkMax m_motorElevatorLeft = new CANSparkMax(ElevatorConstants.leftMotorID, MotorType.kBrushless);
  private final CANSparkMax m_motorElevatorRight = new CANSparkMax(ElevatorConstants.rightMotorID, MotorType.kBrushless);
  private final DigitalInput leftUpLimitSwitch = new DigitalInput(ElevatorConstants.leftUpChannel);
  private final DigitalInput leftDownLimitSwitch = new DigitalInput(ElevatorConstants.leftDownChannel);
  private final DigitalInput rightUpLimitSwitch = new DigitalInput(ElevatorConstants.rightUpChannel);
  private final DigitalInput rightDownLimitSwitch = new DigitalInput(ElevatorConstants.rightDownChannel);

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_motorElevatorLeft.setInverted(false);
    m_motorElevatorRight.setInverted(true);
    // m_motorElevatorRight.follow(m_motorElevatorLeft, true);
    m_motorElevatorLeft.follow(m_motorElevatorRight, true);
    m_motorElevatorLeft.getEncoder().setPosition(0);
    m_motorElevatorRight.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void elevatorRunUp(double speed){
    if(this.atUpLimit() == false) m_motorElevatorLeft.set(speed);
    else m_motorElevatorLeft.set(0);
  }

  public void elevatorRunDown(double speed){
    if(this.atDownLimit() == false) m_motorElevatorLeft.set(speed);
    else m_motorElevatorLeft.set(0);
  }
  
  public void elevatorstop(){
    m_motorElevatorLeft.set(0);
  }

  public double getLeftMotorPos(){
    return m_motorElevatorLeft.getEncoder().getPosition();
  }

  public double getLeftMotorVel(){
    return m_motorElevatorLeft.getEncoder().getVelocity();
  }

  public double getRightMotorPos(){
    return m_motorElevatorRight.getEncoder().getPosition();
  }

  public double getRightMotorVel(){
    return m_motorElevatorRight.getEncoder().getVelocity();
  }

  public void resetEncoders(){
    m_motorElevatorLeft.getEncoder().setPosition(0);
    m_motorElevatorRight.getEncoder().setPosition(0);
  }

  public boolean getLeftUpLimit(){
    return leftUpLimitSwitch.get();
  }

  public boolean getLeftDownLimit(){
    return leftDownLimitSwitch.get();
  }

  public boolean getRightUpLimit(){
    return rightUpLimitSwitch.get();
  }

  public boolean getRightDownLimit(){
    return rightDownLimitSwitch.get();
  }

  public boolean atLimit(){
    if(this.getLeftUpLimit() == false && this.getLeftDownLimit() == false
    && this.getRightUpLimit() == false && this.getRightDownLimit() == false){
      return false;
    } else return true;
  }

  public boolean atUpLimit(){
   if(this.getLeftUpLimit() == false && this.getRightUpLimit() == false) return false;
   else return true;
  }

  public boolean atDownLimit(){
    if(this.getLeftDownLimit() == false && this.getRightDownLimit() == false) return false;
    else return true;
   }
}
