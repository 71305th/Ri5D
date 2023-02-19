package frc.robot.ChenryLib;

public class PID {
    private double lastError = 0, d = 0, i = 0;
    double output;
    private double kP, kI, kD, windup, limit;

    public PID(double ikP, double ikI, double ikD, double iwindup, double ilimit){
        kP = ikP;
        kI = ikI;
        kD = ikD;
        windup = iwindup;
        limit = ilimit;
    }

    public double calculate (double error){
        if (Math.abs(error) <= windup) i += error; else i= 0;
        double iOut = MathUtility.clamp(i * kI, -limit, limit);
        d = error - lastError;
        lastError = error;
        output = (error * kP) + iOut + (d * kD);
        return output;
    }
}
