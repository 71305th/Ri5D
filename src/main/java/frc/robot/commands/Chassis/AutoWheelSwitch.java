package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberWheelSubsystem;

public class AutoWheelSwitch extends CommandBase{
    private final GrabberWheelSubsystem wheel;
    
    private int state;
    private double time;

    private final Timer timer = new Timer();

    public AutoWheelSwitch(GrabberWheelSubsystem m_wheel, int state, double time){
        this.wheel = m_wheel;
        this.state =  state;
        this.time = time;
        addRequirements(m_wheel);
    }

    public void initialize(){
        timer.reset();
        timer.start();
    }

    public void execute(){
        wheel.rollRun(state * 0.2);
    }

    public void end(boolean interrupted){
        wheel.rollRun(0);
    }

    public boolean isFinished(){
        if(timer.get() > this.time) return true;
        return false;
    }

}
