// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Chassis.AutoArmMove;
import frc.robot.commands.Chassis.AutoElbowMove;
import frc.robot.commands.Chassis.AutoElevatorMove;
import frc.robot.commands.Chassis.AutoMove;
import frc.robot.commands.Chassis.AutoWheelSwitch;
import frc.robot.commands.Grabber.GrabAndRelease;
import frc.robot.commands.Grabber.WheelEject;
import frc.robot.commands.Grabber.WheelIntake;


import frc.robot.commands.Grabber.WheelsTurnAndStop;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.GrabberPCMSubsystem;
import frc.robot.subsystems.GrabberWheelSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Joystick
  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverController);
  private final Joystick operatorJoystick = new Joystick(OIConstants.kOperatorController);

  // Subsystems
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final ElbowSubsystem m_elbow = new ElbowSubsystem();
  private final GrabberPCMSubsystem m_grabPCM = new GrabberPCMSubsystem();
  private final GrabberWheelSubsystem m_grabWheel = new GrabberWheelSubsystem();

  // Commands
  //private final LockPID m_setPoint = new LockPID(m_drive);
  private final GrabAndRelease m_grabAndRelease = new GrabAndRelease(m_grabPCM);

  // PID Controller
  PIDController pidController = new PIDController(0.05, 0, 0);

  // Elbow pos
  private double pos;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Drive
    m_drive.setDefaultCommand(new RunCommand(() -> {
      if (driverJoystick.getRawButtonPressed(OIConstants.Btn_RB))
      m_drive.arcadeDrive(
        -driverJoystick.getRawAxis(OIConstants.leftStick_Y) * DriveConstants.chassisArcadeSpdScaler * 0.5, 
        driverJoystick.getRawAxis(OIConstants.rightStick_X) * DriveConstants.chassisArcadeRotScaler* 0.5);
      else
      m_drive.arcadeDrive(
        -driverJoystick.getRawAxis(OIConstants.leftStick_Y) * DriveConstants.chassisArcadeSpdScaler, 
        driverJoystick.getRawAxis(OIConstants.rightStick_X) * DriveConstants.chassisArcadeRotScaler);
    }, m_drive));


    // Elevator
    m_elevator.setDefaultCommand(new RunCommand(() -> {
      m_elevator.set(operatorJoystick.getRawAxis(OIConstants.rightStick_Y));
    } , m_elevator));

    // Arm
    m_arm.setDefaultCommand(new RunCommand(() -> {
      if(operatorJoystick.getRawAxis(OIConstants.trigger_L) > 0.05){
        m_arm.set(-operatorJoystick.getRawAxis(OIConstants.trigger_L) * ArmConstants.kArmSpeedScaler);
      }else if (operatorJoystick.getRawAxis(OIConstants.trigger_R) > 0.05){
        m_arm.set(operatorJoystick.getRawAxis(OIConstants.trigger_R) * ArmConstants.kArmSpeedScaler);
      }
    }, m_arm));

    // Elbow
    m_elbow.setDefaultCommand(new RunCommand(() -> {
      if(operatorJoystick.getRawAxis(OIConstants.leftStick_Y) > 0.05 || operatorJoystick.getRawAxis(OIConstants.leftStick_Y) < -0.05){
        m_elbow.set(operatorJoystick.getRawAxis(OIConstants.leftStick_Y) * ElbowConstants.kElbowSpeedScaler);
        pos = m_elbow.getPosition();
      } else {
        m_elbow.set(pidController.calculate(m_elbow.getPosition(), pos));
      }
    }, m_elbow));

    // Configure the button bindings
    configureButtonBindings();
    m_drive.zeroHeading();
  }

  private void configureButtonBindings() {
    // make the grabber grab and release
    new JoystickButton(operatorJoystick, OIConstants.Btn_LB).onTrue(m_grabAndRelease);

    new JoystickButton(operatorJoystick, OIConstants.Btn_Y).whileTrue(new WheelEject(m_grabWheel));
    new JoystickButton(operatorJoystick, OIConstants.Btn_RB).whileTrue(new WheelIntake(m_grabWheel));

    // make the wheels on the grabber turn and stop
    // new JoystickButton(operatorJoystick, OIConstants.Btn_RB).onTrue(m_wheelsTurnAndStop); 已在Subsystem中

    // make the elevator go up or down in a click
    // new JoystickButton(operatorJoystick, OIConstants.Btn_X).onTrue(m_oneButtonRunUpDown); 已在Subsystem中
  }

  public Command getAutonomousCommand() {
    return 
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          new AutoWheelSwitch(m_grabWheel, 1, 1),
          new AutoElbowMove(m_elbow, 0.5),
          new AutoElevatorMove(m_elevator, 0),
          new AutoArmMove(m_arm, 0)
        ),
        new AutoWheelSwitch(m_grabWheel, -1, 1),
        new ParallelCommandGroup(
          new AutoWheelSwitch(m_grabWheel, 1, 1),
          new AutoMove(m_drive, 5),
          new AutoElbowMove(m_elbow, 0.5),
          new AutoElevatorMove(m_elevator, 0),
          new AutoArmMove(m_arm, 0)
        ),
        new ParallelCommandGroup(
          new AutoWheelSwitch(m_grabWheel, 1, 1),
          new AutoMove(m_drive, 5),
          new AutoElbowMove(m_elbow, 0.5),
          new AutoElevatorMove(m_elevator, 0),
          new AutoArmMove(m_arm, 0),
          new WheelsTurnAndStop(m_grabWheel)
        ),
        new AutoWheelSwitch(m_grabWheel, -1, 1)
      );
  }

  public void testMotor(){
    this.m_drive.testMotor();
  }
}