// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Chassis.PathFollowingRamsete;
import frc.robot.commands.Grabber.AutoGrabClose;
import frc.robot.commands.Grabber.AutoGrabOpen;
import frc.robot.commands.Grabber.GrabAndRelease;
import frc.robot.commands.Chassis.AutoElbowMove;
import frc.robot.commands.Chassis.AutoMove;
import frc.robot.commands.Chassis.AutoRotate;
import frc.robot.commands.Chassis.LockPID;
import frc.robot.commands.Elevator.OneButtonRunUpDown;
import frc.robot.commands.Grabber.WheelsTurnAndStop;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.GrabberPCMSubsystem;
import frc.robot.subsystems.GrabberWheelSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final LockPID m_setPoint = new LockPID(m_drive);
  private final GrabAndRelease m_grabAndRelease = new GrabAndRelease(m_grabPCM);
  private final WheelsTurnAndStop m_wheelsTurnAndStop = new WheelsTurnAndStop(m_grabWheel);
  private final OneButtonRunUpDown m_oneButtonRunUpDown = new OneButtonRunUpDown(m_elevator);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Drive
    m_drive.setDefaultCommand(new RunCommand(() -> {
      m_drive.arcadeDrive(
        -driverJoystick.getRawAxis(OIConstants.leftStick_Y) * DriveConstants.kChassisArcadeSpdScaler, 
        driverJoystick.getRawAxis(OIConstants.rightStick_X) * DriveConstants.kChassisArcadeRotScaler);
    }, m_drive));


    // Elevator
    m_elevator.setDefaultCommand(new RunCommand(() -> {
      if (operatorJoystick.getRawAxis(OIConstants.rightStick_Y) < -0.3) {
        m_elevator.elevatorRunUp(ElevatorConstants.kElevatorUpSpeedScaler);
      } else if (operatorJoystick.getRawAxis(OIConstants.rightStick_Y) > 0.3) {
        m_elevator.elevatorRunDown(ElevatorConstants.kElevatorDownSpeedScaler);
      } 
    } , m_elevator));

    // Arm
    m_arm.setDefaultCommand(new RunCommand(() -> {
      if(operatorJoystick.getRawAxis(OIConstants.trigger_L) > 0.05){
        m_arm.run(operatorJoystick.getRawAxis(OIConstants.trigger_L) * ArmConstants.kArmSpeedScaler);
      }else{
        m_arm.run(-operatorJoystick.getRawAxis(OIConstants.trigger_R) * ArmConstants.kArmSpeedScaler);
      }
    }, m_arm));

    // Elbow
    m_elbow.setDefaultCommand(new RunCommand(() -> {
      m_elbow.elbowRun(-operatorJoystick.getRawAxis(OIConstants.leftStick_Y) * ElbowConstants.kElbowSpeedScaler);
    }, m_elbow));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(driverJoystick, OIConstants.Btn_A).onTrue(m_setPoint);
    // new JoystickButton(driverJoystick, OIConstants.Btn_B).onTrue(new RunCommand( () -> {m_drive.resetEncoders();}, m_drive));

    // make the grabber grab and release
    new JoystickButton(operatorJoystick, OIConstants.Btn_LB).onTrue(m_grabAndRelease);

    // make the wheels on the grabber turn and stop
    new JoystickButton(operatorJoystick, OIConstants.Btn_RB).onTrue(m_wheelsTurnAndStop);

    // make the elevator go up or down in a click
    new JoystickButton(operatorJoystick, OIConstants.Btn_X).onTrue(m_oneButtonRunUpDown);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new SequentialCommandGroup(
    //   new PathFollowingRamsete(m_drive, "New Path", true), 
    //   m_setPoint);
    return new SequentialCommandGroup(
      new AutoMove(m_drive, 5),
      new AutoGrabOpen(m_grabPCM, m_grabWheel),
      new AutoElbowMove(m_elbow, 0.5),
      new AutoGrabClose(m_grabPCM, m_grabWheel),
      new AutoElbowMove(m_elbow, 0),
      new AutoMove(m_drive, 0),
      new AutoRotate(m_drive, -180),
      new AutoElbowMove(m_elbow, 0.2),
      new AutoGrabOpen(m_grabPCM, m_grabWheel)
    );
  }

  public void testMotor(){
    this.m_drive.testMotor();
  }
}