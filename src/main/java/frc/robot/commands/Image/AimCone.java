package frc.robot.commands.Image;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;

public class AimCone extends CommandBase{
    
    private final LimelightSubsystem limelight;

    public AimCone(LimelightSubsystem limelight){
        this.limelight = limelight;
        addRequirements(limelight);
    }

    public Translation3d result = new Translation3d(0, 0, 0);
    double tx;
    double ty;
    double dis;
    double kx = 0.1;
    double ky = 0.1;
    double kz = 0.1;

    @Override
    public void initialize() {
        result = new Translation3d(0,0,0);
    }

    @Override
    public void execute() {
        tx = limelight.getX_2();
        ty = limelight.getY_2();
        dis = Math.sqrt(Math.pow(tx, 2) + Math.pow(ty, 2));

        result = new Translation3d(kx*tx, ky*ty, kz*dis);
    }

    @Override
    public void end(boolean interrupted) {
        
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }

}
