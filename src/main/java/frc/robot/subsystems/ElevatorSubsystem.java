// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  private final CANSparkMax m_motorElevatorLeft = new CANSparkMax(ElevatorConstants.leftMotorID, MotorType.kBrushless);
  private final CANSparkMax m_motorElevatorRight = new CANSparkMax(ElevatorConstants.rightMotorID, MotorType.kBrushless);
  private final DigitalInput leftUpLimitSwitch = new DigitalInput(ElevatorConstants.leftUpChannel);
  private final DigitalInput leftDownLimitSwitch = new DigitalInput(ElevatorConstants.leftDownChannel);
  private final DigitalInput rightUpLimitSwitch = new DigitalInput(ElevatorConstants.rightUpChannel);
  private final DigitalInput rightDownLimitSwitch = new DigitalInput(ElevatorConstants.rightDownChannel);

  private final MotorControllerGroup elevator = new MotorControllerGroup(m_motorElevatorLeft, m_motorElevatorRight);

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_motorElevatorLeft.setInverted(true);
    elevator.setInverted(false);
    resetEncoders();
  }

  public void set(double speed){
    if(!this.atLimit())elevator.set(speed);
    else elevator.set(0);
  }
  
  public void stop(){
    m_motorElevatorLeft.set(0);
  }

  public double getLeftMotorPos(){
    return m_motorElevatorLeft.getEncoder().getPosition();
  }

  public double getRightMotorPos(){
    return m_motorElevatorRight.getEncoder().getPosition();
  }

  public double getPos() {
    return (getLeftMotorPos() + getRightMotorPos())/2;
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
    if(!(atDownLimit() || atUpLimit()))return false;
    else return true;
  }

  public boolean atUpLimit(){
    if (this.getLeftUpLimit() || this.getRightUpLimit()) return true;
    else return false;
  }

  public boolean atDownLimit(){
    if (this.getLeftDownLimit() || this.getRightDownLimit()) return true;
    else return false;
  }
  
  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
