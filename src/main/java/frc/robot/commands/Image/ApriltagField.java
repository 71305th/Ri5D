package frc.robot.commands.Image;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.field;
import frc.robot.subsystems.ApriltagSubsystem;

public class ApriltagField extends CommandBase{
    
    private final ApriltagSubsystem apriltag;

    public ApriltagField(ApriltagSubsystem apriltag){
        this.apriltag = apriltag;
        addRequirements(apriltag);
    }
    /**
     * In meters
     */
    public Translation2d position = new Translation2d();
    int targetID;
    Translation2d IDposition;
    Transform3d error;

    @Override
    public void initialize() {
        targetID = 0;
    }

    @Override
    public void execute() {
       targetID = apriltag.getTargetID();

       if(targetID != 0 && targetID >= 5){
       IDposition =  field.fieldmMap.get(targetID);
       error = apriltag.getCameratoTarget();

       position = new Translation2d(IDposition.getX() + field.kx_dis*error.getX(), IDposition.getY() - field.ky_dis*error.getY());
       }

       if(targetID != 0 && targetID <= 4){
        IDposition =  field.fieldmMap.get(targetID);
        error = apriltag.getCameratoTarget();

        position = new Translation2d(IDposition.getX() - field.kx_dis*error.getX(), IDposition.getY() + field.ky_dis*error.getY());
       }

       SmartDashboard.putNumber("position_x", position.getX());
       SmartDashboard.putNumber("position_y", position.getY());
       
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
