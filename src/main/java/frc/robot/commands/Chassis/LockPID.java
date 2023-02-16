// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class LockPID extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final DriveSubsystem drive;
  private double lastError = 0;
  private boolean isEnd = false;
  private double lasttime = 0;
  private double initL = 0;
  private double initR = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LockPID(DriveSubsystem drive) {
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setMotor2zero();
    initL = drive.getLeftRelativeDistance();
    initR = drive.getRightRelativeDistance();
    isEnd = false;
    // lockPIDLeft.setTolerance(0, 0);
    // lockPIDRight.setTolerance(0, 0);
    System.out.println("LockPID Enabled");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Joystick driverJoystick = new Joystick(0);

    if ( Math.abs(driverJoystick.getRawAxis(OIConstants.leftStick_Y)) > 0.1 
    || Math.abs(driverJoystick.getRawAxis(OIConstants.rightStick_X)) > 0.1 ){
      stop();
    } 

    double distLeft = drive.getLeftRelativeDistance();
    double distRight = drive.getRightRelativeDistance();

    double trueL = distLeft - initL;
    double trueR = distRight - initR;

    System.out.println("dL" + distLeft);
    System.out.println("dR" + distRight);

    double LeftOutput = PID(PIDConstants.kP_Lock, PIDConstants.kI_Lock, PIDConstants.kD_Lock, PIDConstants.iLimit_Lock, trueL, 0);
    double RightOutput = PID(PIDConstants.kP_Lock, PIDConstants.kI_Lock, PIDConstants.kD_Lock, PIDConstants.iLimit_Lock, trueR, 0);

    drive.setLeftSpeed( LeftOutput );
    drive.setRightSpeed( RightOutput );

    // System.out.print("LeftOutput : ");
    // System.out.println(LeftOutput);

    // System.out.print("RightOutput : ");
    // System.out.println(RightOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stop();
    System.out.println("LockPID Disabled");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isEnd;
  }

  public double PID (double kP, double kI, double kD, double iLimit, double ctrPos, double target) {
    double output = 0;
    double error = target - ctrPos;
    double i = 0;
    double time = Timer.getFPGATimestamp();
    double deltaT = time - lasttime;
    //double deltaError = error / deltaT;
    double d = (lastError - error) / deltaT;
    // double rate = ( drive.getLeftVelocity() + drive.getRightVelocity() ) / 2;

    if (error < Math.abs(iLimit)) i += error;
    else i = 0;

    output = kP * error + kI * i + kD * d;
    lasttime = time;
    lastError = error;
    System.out.println("error" + error);
    return output;
  }

  public void stop() {
    isEnd = true;
  }
}