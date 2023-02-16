package frc.robot.commands.Chassis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoMove  extends CommandBase{
    private final DriveSubsystem drive;
    
    private PIDController pidCon = new PIDController(0, 0, 0);
    
    private double targetPos;

    public AutoMove(DriveSubsystem m_drive, double targetPos){
        this.drive = m_drive;
        this.targetPos = targetPos;
        addRequirements(m_drive);
    }

    public void initialize(){
    }

    public void execute(){
        double output = pidCon.calculate(this.drive.getPose().getX(), this.targetPos);
        this.drive.arcadeDrive(output, 0);
    }

    public void end(boolean interrupted){
        this.drive.arcadeDrive(0, 0);
    }

    public boolean isFinished(){
        if(Math.abs(this.targetPos-this.drive.getPose().getX())<0.05){
            this.drive.arcadeDrive(0, 0);
            return true;
        }
        return false;
    }

}
