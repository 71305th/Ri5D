package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase{
    
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTable table_2 = NetworkTableInstance.create().getTable("limelight");
    
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry ta = table.getEntry("ta");

    NetworkTableEntry tx_2 = table_2.getEntry("tx");
    NetworkTableEntry ty_2 = table_2.getEntry("ty");
    NetworkTableEntry tv_2 = table_2.getEntry("tv");
    NetworkTableEntry ta_2 = table_2.getEntry("ta");
    
    double x, x_2;
    double y, y_2;
    double a, a_2;
    boolean v, v_2;

    @Override
    public void periodic(){

        x = tx.getDouble(0);
        y = ty.getDouble(0);
        a = ta.getDouble(0);
        v = tv.getBoolean(false);

        x_2 = tx_2.getDouble(0);
        y_2 = ty_2.getDouble(0);
        a_2 = ta_2.getDouble(0);
        v_2 = tv_2.getBoolean(false);
    
        SmartDashboard.putNumber("tx", x);
        SmartDashboard.putNumber("ty", y);
        SmartDashboard.putNumber("ta", a);
        SmartDashboard.putBoolean("tv", v);

        SmartDashboard.putNumber("tx_2", x_2);
        SmartDashboard.putNumber("ty_2", y_2);
        SmartDashboard.putNumber("ta_2", a_2);
        SmartDashboard.putBoolean("tv_2", v_2);

    }

    /**
     * Cone_1
     */
    public double getX_1(){
        return x;
    }
    /**
     * Cone_1
     */
    public double getY_1(){
        return y;
    }
    /**
     * Cone_1
     */
    public boolean getV_1(){
        return v;
    }
    /**
     * Cone_2
     */
    public double getX_2(){
        return x_2;
    }
  /**
     * Cone_2
     */
    public double getY_2(){
        return y_2;
    }
  /**
     * Cone_2
     */
    public boolean getV_2(){
        return v_2;
    }
}
