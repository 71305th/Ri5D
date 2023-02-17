package frc.robot.commands.Image;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Cones;
import frc.robot.subsystems.LimelightSubsystem;

public class CheckCones extends CommandBase{
    
    private final LimelightSubsystem limelight;

    public CheckCones(LimelightSubsystem limelight){
        this.limelight = limelight;

        addRequirements(limelight);
    }

    double tangentCone_1 = 0;
    double tangentCone_2 = 0;
    double limeLightTheta = 0;
    double limeLightHight = 0;
    double cone_2hightTheorectic;
    
    double adjustConstant1 = 2; //limelight degrees
    double adjustConstant2 = 0.2; //meters
    double startTime;
    double currentTime;
    double timeError = 3; //updates Boolean Cone_2Empty in every 3 seconds
    double tyErrorSum;
    double heightErrorSum;

    public boolean Cone_2Empty;

    @Override
    public void initialize() {
        Cone_2Empty = false;
        startTime = Timer.getFPGATimestamp();
        tyErrorSum = 0;
        heightErrorSum = 0;
        currentTime = 0;

    }

    @Override
    public void execute() {
        currentTime = Timer.getFPGATimestamp();

        tangentCone_1 = Math.abs(Math.tan(limelight.getY_1()*Math.PI/180 + limeLightTheta*Math.PI/180 + Math.PI/2));
        tangentCone_2 = Math.abs(Math.tan(limelight.getY_2()*Math.PI/180 + limeLightTheta*Math.PI/180 + Math.PI/2));
        cone_2hightTheorectic = Cones.distenceBetweenCones*tangentCone_2 + Cones.conesHight_1*tangentCone_2/tangentCone_1;

        tyErrorSum += (Math.abs(limelight.getY_1() - limelight.getY_2()));
        heightErrorSum += (Math.abs(cone_2hightTheorectic - Cones.conesHight_2));


        if(currentTime - startTime == timeError){
            if(tyErrorSum < adjustConstant1){
                if(heightErrorSum < adjustConstant2){
                    Cone_2Empty = true;
                }else{
                    Cone_2Empty = false;
                }
            }else{
                Cone_2Empty = true;
             }
        
             startTime = Timer.getFPGATimestamp();
        }

        SmartDashboard.putBoolean("Cone2Empty", Cone_2Empty);

        
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}

