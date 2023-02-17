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
import frc.robot.subsystems.ApriltagSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.GrabberPCMSubsystem;
import frc.robot.subsystems.GrabberWheelSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
  private final Joystick mDriverJoystick = new Joystick(OIConstants.kDriverController);
  private final Joystick mOperatorJoystick = new Joystick(OIConstants.kOperatorController);

  // Subsystems
  private final DriveSubsystem mDrive = new DriveSubsystem();
  private final ElevatorSubsystem mElevator = new ElevatorSubsystem();
  private final ArmSubsystem mArm = new ArmSubsystem();
  private final ElbowSubsystem mElbow = new ElbowSubsystem();
  private final GrabberPCMSubsystem mGrabPCM = new GrabberPCMSubsystem();
  private final GrabberWheelSubsystem mGrabWheel = new GrabberWheelSubsystem();
  private final ApriltagSubsystem mApriltag = new ApriltagSubsystem();
  private final LimelightSubsystem mLimelight = new LimelightSubsystem();

  // Commands
  private final LockPID mSetPoint = new LockPID(mDrive);
  private final GrabAndRelease mGrabAndRelease = new GrabAndRelease(mGrabPCM);
  private final WheelsTurnAndStop mWheelsTurnAndStop = new WheelsTurnAndStop(mGrabWheel);
  private final OneButtonRunUpDown mOneButtonRunUpDown = new OneButtonRunUpDown(mElevator);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Drive
    mDrive.setDefaultCommand(new RunCommand(() -> {
      mDrive.arcadeDrive(
        -mDriverJoystick.getRawAxis(OIConstants.leftStick_Y) * DriveConstants.kChassisArcadeSpdScaler, 
        mDriverJoystick.getRawAxis(OIConstants.rightStick_X) * DriveConstants.kChassisArcadeRotScaler);
    }, mDrive));


    // Elevator
    mElevator.setDefaultCommand(new RunCommand(() -> {
      if ( mOperatorJoystick.getRawAxis(OIConstants.rightStick_Y) < -0.3) {
        mElevator.elevatorRunUp(ElevatorConstants.kElevatorUpSpeedScaler);
      } else if ( mOperatorJoystick.getRawAxis(OIConstants.rightStick_Y) > 0.3) {
        mElevator.elevatorRunDown(ElevatorConstants.kElevatorDownSpeedScaler);
      } 
    } , mElevator));

    // Arm
    mArm.setDefaultCommand(new RunCommand(() -> {
      if( mOperatorJoystick.getRawAxis(OIConstants.trigger_L) > 0.05){
        mArm.run( mOperatorJoystick.getRawAxis(OIConstants.trigger_L) * ArmConstants.kArmSpeedScaler);
      }else{
        mArm.run(-mOperatorJoystick.getRawAxis(OIConstants.trigger_R) * ArmConstants.kArmSpeedScaler);
      }
    }, mArm));

    // Elbow
    mElbow.setDefaultCommand(new RunCommand(() -> {
      mElbow.elbowRun( -mOperatorJoystick.getRawAxis(OIConstants.leftStick_Y) * ElbowConstants.kElbowSpeedScaler );
    }, mElbow));

    //apriltag
    mApriltag.setDefaultCommand(new RunCommand(()->{
      if(mOperatorJoystick.getRawButtonPressed(OIConstants.Btn_B)){
        mApriltag.getPosByApriltag();
      }
    },mApriltag));

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
    new JoystickButton(mOperatorJoystick, OIConstants.Btn_LB).onTrue(mGrabAndRelease);

    // make the wheels on the grabber turn and stop
    new JoystickButton(mOperatorJoystick, OIConstants.Btn_RB).onTrue(mWheelsTurnAndStop);

    // make the elevator go up or down in a click
    new JoystickButton(mOperatorJoystick, OIConstants.Btn_X).onTrue(mOneButtonRunUpDown);
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


    return 
      new SequentialCommandGroup( 
      new AutoMove(mDrive, 5),
      new AutoGrabOpen(mGrabPCM, mGrabWheel),
      new AutoElbowMove(mElbow, 0.5),
      new AutoGrabClose(mGrabPCM, mGrabWheel),
      new AutoElbowMove(mElbow, 0),
      new AutoMove(mDrive, 0),
      new AutoRotate(mDrive, -180),
      new AutoElbowMove(mElbow, 0.2),
      new AutoGrabOpen(mGrabPCM, mGrabWheel));
  }

  public void testMotor(){
    this.mDrive.testMotor();
  }
}