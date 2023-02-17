// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class OneButtonRunUpDown extends CommandBase {

  private final ElevatorSubsystem elevatorSubsystem;
  private boolean state;

  /** Creates a new OneButtonRunUpDown. */
  public OneButtonRunUpDown(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.elevatorstop();
    state = true; //true = up, false = down
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(state) {
      if(elevatorSubsystem.atUpLimit()){
        elevatorSubsystem.elevatorstop();
      }else{
        elevatorSubsystem.elevatorRunUp(ElevatorConstants.kElevatorUpSpeedScaler);
      }
    } else {
      if(elevatorSubsystem.atDownLimit()){
        elevatorSubsystem.elevatorstop();
      }else{
        elevatorSubsystem.elevatorRunDown(ElevatorConstants.kElevatorDownSpeedScaler);
      }
    }
    state = !state;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
