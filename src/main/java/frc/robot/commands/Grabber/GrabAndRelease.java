// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberPCMSubsystem;

public class GrabAndRelease extends CommandBase {

  private final GrabberPCMSubsystem grabPCM;
  private boolean state = false;

  /** Creates a new GrabAndRelease. */
  public GrabAndRelease(GrabberPCMSubsystem grabPCM) {
    this.grabPCM = grabPCM;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(grabPCM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(state == true) grabPCM.handOpen();
    else grabPCM.handClose();
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
