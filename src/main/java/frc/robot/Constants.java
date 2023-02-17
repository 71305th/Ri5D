// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        // Motors
        public static final int kMotorFrontLeft = 2;
        public static final int kMotorFrontRight = 3;
        public static final int kMotorRearLeft = 1;
        public static final int kMotorRearRight = 4;

        // Encoders
        // public static final int kLeftEncoderPort = 3;
        // public static final int kRightEncoderPort = 4;
        public static final double kEncoderCPR = 42;
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kWheelCircumference = kWheelDiameterMeters * Math.PI;
        public static final double kGearRatio = 25 / 12;
        public static final double kDistancePerPulse = kWheelCircumference / kEncoderCPR / kGearRatio;
        // distancePerPulse = wheelPerimeter / EncoderCPR

        // Trajectory Constraints
        public static final double ksVolts = 2.4;
        public static final double kvVoltSecondsPerMeter = 7.2;
        public static final double kaVoltSecondsSquaredPerMeter = 0.0;

        // Chassis Related
        public static final double kTrackWidthmeters = 0.595;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackWidthmeters);
        public static final double kChassisArcadeSpdScaler = 1;
        public static final double kChassisArcadeRotScaler = 1;
        public static final double kChassisTankScaler = 0.5;
        public static final double kRobotDriveHight = 0.20;
    }

    public static final class PIDConstants {
        // Charge Station
        public static final double kP_Lock = 1.0;
        public static final double kI_Lock = 0.05;
        public static final double kD_Lock = 0.000;
        public static final double kILimit_Lock = 0.36;
    }

    public static final class AutoConstants {
        // Feed forward shit
        public static final double kRamseteB = 2.5;
        public static final double kRamseteZeta = 3.0;

        // Positions
        public static final double kMaxSpeedMetersPerSecond = 1.86;
        public static final double kMaxAccelerationMetersPerSecondSquared = 5.01;
    }

    public static final class ArmConstants {
        public static final double kArmSpeedScaler = 0.3;
        public static final double kArmOneLength = 0.80;
        public static final int kMotorID = 14;

        public static final double kEncoderCPR = 42;
        public static final double kGearRatio = 36 * 72 / 48;
    }

    public static final class ElbowConstants {
        public static final int kMotorID = 12;
        public static final int kCancoderID = 0;
        public static final double kElbowSpeedScaler = 0.3;

        public static final double kEncoderCPR = 42;
        public static final double kGearRatio = 40 * 72 / 20;
    }

    public static final class GrabberConstants {
        public static final int kCompressorID = 0;
        public static final int kForwardChannel = 6;
        public static final int kReverseChannel = 7;
        public static final int kMotorID = 15;
    }

    public final class ElevatorConstants {
        public static final int kLeftMotorID = 11;
        public static final int kRightMotorID = 8;
        public static final double kElevatorUpSpeedScaler = 0.5;
        public static final double kElevatorDownSpeedScaler = 0.3;
        public static final int kLeftUpChannel = 0;
        public static final int kLeftDownChannel = 1;
        public static final int kRightUpChannel = 2;
        public static final int kRightDownChannel = 3;
    }

    public static final class OIConstants {
        public static final int kDriverController = 0;
        public static final int kOperatorController = 1;

        public static final int leftStick_X = 0;
        public static final int leftStick_Y = 1;
        public static final int trigger_L = 2;
        public static final int trigger_R = 3;
        public static final int rightStick_X = 4;
        public static final int rightStick_Y = 5;
        public static final int Btn_A = 1;
        public static final int Btn_B = 2;
        public static final int Btn_X = 3;
        public static final int Btn_Y = 4;
        public static final int Btn_LB = 5;
        public static final int Btn_RB = 6;
        public static final int Btn_LS = 9;
        public static final int Btn_RS = 10;
    }

    /**
     * measurements of the cone area in meters:
     * 
     * <ul>
     * <li>cone1 --> lower one 
     * <li>cone2 --> upper one
     */
    public static class ImagineConstants{
        // Apriltag
        public static final double kXDis = 0.1;
        public static final double kYDis = 0.1;
        public static final double kApriltagHeight = 0.515;
      
        // Cone & Apriltag Height
        public static final double kConesHight_2 = 1.06600625;
        public static final double kConesHight_1 = 0.568325;
        public static final double kDistenceBetweenCones = 0.4318;
      }

      public static final class FieldConstants {
        /**
         * length: meters </p>
         * (0,0) --> blue aliance
         * @param targetID
         * @param position
         */
        public static Map<Integer, Translation2d> fieldmMap = new HashMap<>(){{
            put(1, new Translation2d(15.69085, 1.597025));
            put(2, new Translation2d(15.69085, 4.054475));
            put(3, new Translation2d(15.69085, 6.156325));
            put(4, new Translation2d(16.18615, 6.7262375));
            put(5, new Translation2d(0.3556, 6.7262375));
            put(6, new Translation2d(0.8509, 6.156325));
            put(7, new Translation2d(0.8509, 4.054475));
            put(8, new Translation2d(0.8509, 1.597025));
        }};
     }
}