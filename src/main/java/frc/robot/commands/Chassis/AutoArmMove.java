package frc.robot.commands.Chassis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class AutoArmMove extends CommandBase{
    private final ArmSubsystem arm;
    
    private PIDController pidCon = new PIDController(0.05, 0, 0);
    
    private double targetPos;

    public AutoArmMove(ArmSubsystem m_arm, double targetPos){
        this.arm = m_arm;
        this.targetPos = targetPos;
        addRequirements(m_arm);
    }

    public void initialize(){
    }

    public void execute(){
        double output = pidCon.calculate(this.arm.getEncoderPos(), this.targetPos);
        this.arm.set(output);
    }

    public void end(boolean interrupted){
        this.arm.stop();
    }

    public boolean isFinished(){
        if(Math.abs(this.targetPos-arm.getEncoderPos())<0.05){
            this.arm.stop();
            return true;
        }
        return false;
    }

}
