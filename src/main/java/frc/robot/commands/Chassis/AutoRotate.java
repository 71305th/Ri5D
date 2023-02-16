package frc.robot.commands.Chassis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoRotate  extends CommandBase{
    private final DriveSubsystem drive;
    
    private PIDController pidCon = new PIDController(0, 0, 0);
    
    private double targetPos;

    public AutoRotate(DriveSubsystem m_drive, double targetPos){
        this.drive = m_drive;
        this.targetPos = targetPos;
        addRequirements(m_drive);
    }

    public void initialize(){
    }

    public void execute(){
        double output = pidCon.calculate(this.drive.getHeading(), this.targetPos);
        this.drive.arcadeDrive(0, output);
    }

    public void end(boolean interrupted){
        this.drive.arcadeDrive(0, 0);
    }

    public boolean isFinished(){
        if(Math.abs(this.targetPos-this.drive.getHeading())<0.05){
            this.drive.arcadeDrive(0, 0);
            return true;
        }
        return false;
    }

}
