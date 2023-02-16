// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberPCMSubsystem;
import frc.robot.subsystems.GrabberWheelSubsystem;

public class AutoGrabClose extends CommandBase {

  private final GrabberPCMSubsystem grabPCM;
  private final GrabberWheelSubsystem grabWheel;

  /** Creates a new GrabAndRelease. */
  public AutoGrabClose(GrabberPCMSubsystem grabPCM, GrabberWheelSubsystem grabWheel) {
    this.grabPCM = grabPCM;
    this.grabWheel = grabWheel;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(grabPCM, grabWheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.grabPCM.handClose();
    this.grabWheel.rollRun(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
