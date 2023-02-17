package frc.robot.commands.Grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberWheelSubsystem;

public class WheelIntake extends CommandBase{

    private final GrabberWheelSubsystem grabWheel;

  /** Creates a new GrabAndRelease. */
  public WheelIntake(GrabberWheelSubsystem grabWheel) {
    this.grabWheel=grabWheel;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(grabWheel);
  }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.grabWheel.rollRun(-0.2);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.grabWheel.rollRun(0);
    }

    public void interrupted(){
        this.grabWheel.rollRun(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
