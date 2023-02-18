package frc.robot.commands.Chassis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class AutoElevatorMove extends CommandBase{
        
    private ElevatorSubsystem elevator;

    private PIDController pidCon = new PIDController(0.05, 0, 0);
    
    private double targetPos;

    public AutoElevatorMove(ElevatorSubsystem m_elevator, double targetPos){
        this.elevator = m_elevator;
        this.targetPos = targetPos;
        addRequirements(m_elevator);
    }

    public void initialize(){
    }

    public void execute(){
        double output = pidCon.calculate(this.elevator.getPos(), this.targetPos);
        this.elevator.set(output);
    }

    public void end(boolean interrupted){
        this.elevator.stop();
    }

    public boolean isFinished(){
        if(Math.abs(this.targetPos-elevator.getPos())<0.05){
            this.elevator.stop();
            return true;
        }
        return false;
    }

}
