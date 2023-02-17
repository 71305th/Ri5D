package frc.robot.subsystems;

import org.apache.commons.lang3.tuple.ImmutableTriple;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ImagineConstants;

public class LimelightSubsystem extends SubsystemBase{
    
    NetworkTable mTable = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTable mTable_2 = NetworkTableInstance.create().getTable("limelight");
    
    NetworkTableEntry tx_1 = mTable.getEntry("tx");
    NetworkTableEntry ty_1 = mTable.getEntry("ty");
    NetworkTableEntry tv_1 = mTable.getEntry("tv");
    NetworkTableEntry ta_1 = mTable.getEntry("ta");

    NetworkTableEntry tx_2 = mTable_2.getEntry("tx");
    NetworkTableEntry ty_2 = mTable_2.getEntry("ty");
    NetworkTableEntry tv_2 = mTable_2.getEntry("tv");
    NetworkTableEntry ta_2 = mTable_2.getEntry("ta");
    
    double mX_1, mY_1, mA_1, mX_2, mY_2, mA_2;
    boolean mV_1, mV_2;

    double tangentCone_1 = 0;
    double tangentCone_2 = 0;
    double limeLightTheta = 0;
    double limeLightHight = 0;
    double cone_2hightTheorectic;
    
    double adjustConstant1 = 2; //limelight degrees
    double adjustConstant2 = 0.2; //meters
    double startTime = 0;
    double currentTime = 0;
    double timeError = 3; //updates Boolean Cone_2Empty in every 3 seconds
    double tyErrorSum = 0;
    double heightErrorSum = 0;

    boolean Cone_2Empty;


    @Override
    public void periodic(){

        mX_1 = tx_1.getDouble(0);
        mY_1 = ty_1.getDouble(0);
        mA_1 = ta_1.getDouble(0);
        mV_1 = tv_1.getBoolean(false);

        mX_2 = tx_2.getDouble(0);
        mY_2 = ty_2.getDouble(0);
        mA_2 = ta_2.getDouble(0);
        mV_2 = tv_2.getBoolean(false);
    
        SmartDashboard.putNumber("tx_1", mX_1);
        SmartDashboard.putNumber("ty_1", mY_1);
        SmartDashboard.putNumber("ta_1", mA_1);
        SmartDashboard.putBoolean("tv_1", mV_1);

        SmartDashboard.putNumber("tx_2", mX_2);
        SmartDashboard.putNumber("ty_2", mY_2);
        SmartDashboard.putNumber("ta_2", mA_2);
        SmartDashboard.putBoolean("tv_2", mV_2);

        currentTime = Timer.getFPGATimestamp();

        tangentCone_1 = Math.abs(Math.tan(ty_1.getDouble(0)*Math.PI/180 + limeLightTheta*Math.PI/180 + Math.PI/2));
        tangentCone_2 = Math.abs(Math.tan(ty_2.getDouble(0)*Math.PI/180 + limeLightTheta*Math.PI/180 + Math.PI/2));
        cone_2hightTheorectic = ImagineConstants.kDistenceBetweenCones*tangentCone_2 + ImagineConstants.kConesHight_1*tangentCone_2/tangentCone_1;

        tyErrorSum += (Math.abs(ty_1.getDouble(0) - ty_2.getDouble(0)));
        heightErrorSum += (Math.abs(cone_2hightTheorectic - ImagineConstants.kConesHight_2));


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
             tyErrorSum = 0;
             heightErrorSum = 0;
        }

        SmartDashboard.putBoolean("Cone2Empty", Cone_2Empty);

        

    }

    /**
     * Cone_1
     */
    public double getX_1(){
        return mX_1;
    }
    
    /**
     * Cone_1
     */
    public double getY_1(){
        return mY_1;
    }

    /**
     * Cone_1
     */
    public double getA_1(){
        return mA_1;
    }

    /**
     * Cone_1
     */
    public boolean getV_1(){
        return mV_1;
    }
    
    /**
     * Cone_2
     */
    public double getX_2(){
        return mX_2;
    }

    /**
     * Cone_2
     */
    public double getY_2(){
        return mY_2;
    }

    /**
     * Cone_2
     */
    public double getA_2(){
        return mA_2;
    }

    /**
     * Cone_2
     */
    public boolean getV_2(){
        return mV_2;
    }

    public boolean isCone_2Empty(){
        return Cone_2Empty;
    }

    
}
