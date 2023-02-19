package frc.robot.ChenryLib;





//gear ratio doesn't work
public class MathUtility {
    //private double encTpr = 42;
    
    //wheel to motor is 50 to 24 and on the 24 its 48 to 12
    //1 rotation on wheel = 2.083 rotation on mid gear
    //2.083 rotation on mid gear = 8.3 rotaions on motor
    //8.3 rotations = 8.3 * 42 ticks
    

    //private double ticksPerWheelRevolution = encTpr * 50 / 24 * 48 / 12;
    private double ticksPerWheelRevolution = 350;
    private double wheelCircumference = 4 * 2.54 * Math.PI / 100 ;

    public MathUtility(){};

    public double encToMeter(double encoderVal){
        return (encoderVal / ticksPerWheelRevolution * wheelCircumference);
    }   
    public double degToRad(double degrees){
        return degrees / 180 * Math.PI;
    }
    public double radToDeg(double radians){
        return radians / Math.PI * 180;
    }

    public static double clamp (double val, double min, double max){
        return Math.max(min, Math.min(max, val));
    }
}
