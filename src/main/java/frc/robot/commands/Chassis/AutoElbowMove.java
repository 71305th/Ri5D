package frc.robot.commands.Chassis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElbowSubsystem;

public class AutoElbowMove  extends CommandBase{
    private final ElbowSubsystem elbow;
    
    private PIDController pidCon = new PIDController(0, 0, 0);
    
    private double targetPos;

    public AutoElbowMove(ElbowSubsystem m_elbow, double targetPos){
        this.elbow = m_elbow;
        this.targetPos = targetPos;
        addRequirements(m_elbow);
    }

    public void initialize(){
    }

    public void execute(){
        double output = pidCon.calculate(this.elbow.getArmAngPos(), this.targetPos);
        this.elbow.elbowRun(output);
    }

    public void end(boolean interrupted){
        this.elbow.elbowStop();
    }

    public boolean isFinished(){
        if(Math.abs(this.targetPos-elbow.getArmAngPos())<0.05){
            this.elbow.elbowStop();;
            return true;
        }
        return false;
    }

}
