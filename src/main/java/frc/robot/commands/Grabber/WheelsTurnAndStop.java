// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberWheelSubsystem;

public class WheelsTurnAndStop extends CommandBase {

  private final GrabberWheelSubsystem grabWheel;
  private boolean state;

  public WheelsTurnAndStop(GrabberWheelSubsystem grabWheel) {
    this.grabWheel = grabWheel;
    addRequirements(grabWheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if(state) 
    grabWheel.rollRun(0.1);
    //else grabWheel.rollStop();
    //state = !state;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    state = !state;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
