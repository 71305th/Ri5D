# Ri5D
 
# v1.0 Remove ApriltagField Replace with `Translation2d getPosByApriltag()` in ApriltagSubsystem

Commiter : **Moyu**

## Features

1. What I Removed

    1. Robot Container

        1. Auto Sepuence
        2. Declaration of ApriltagField
        3. Joystick Button B's Function

            Here is the code i deleted

            ```java
            new JoystickButton(operatorJoystick, OIConstants.Btn_B).onTrue(m_ApriltagField);
            ```

    2. ApriltagField

        Replace with `Translation2d getPosByApriltag()` in ApriltagSubsystem

        * Here is the code : 

        ```java
        /**
         * @return Your Position On Field. If there isn't any target return (-1.0, -1.0)
         **/
        public Translation2d getPosByApriltag(){
            mTargetID = getTargetID();
            Translation2d mApriltagPosition = new Translation2d(-1.0, -1.0);

            if( mTargetID != 0 ){
                mApriltagIDPosition = FieldConstants.fieldmMap.get(mTargetID);
                mError = getCameratoTarget();

                if( mTargetID <= 4 ){
                    mApriltagPosition = new Translation2d (
                            mApriltagIDPosition.getX() + ImagineConstants.kXField * mError.getX(),
                            mApriltagIDPosition.getY() + ImagineConstants.kYField * mError.getY()
                    );
                }else{
                    mApriltagPosition = new Translation2d (
                            mApriltagIDPosition.getX() + ImagineConstants.kXField * mError.getX(),
                            mApriltagIDPosition.getY() - ImagineConstants.kYField * mError.getY()
                    );
                }
            }

            return mApriltagPosition;
        }
        ```

2. Constants Adjust

    1. Cone -> ImagineConstants

        1. Adjust Robot Constants Name

            Change variable name to m[Variable or something might be changed during moving] or k[Constants Name].

            ```java
            // Apriltag
            public static final double kXDis = 0.1;
            public static final double kYDis = 0.1;
            public static final double kApriltagHeight = 0.515;
        
            // Cone & Apriltag Height
            public static final double kConesHight_2 = 1.06600625;
            public static final double kConesHight_1 = 0.568325;
            public static final double kDistenceBetweenCones = 0.4318;
            ```

        2. Move Variable Position

            1. `kArmOneLength` move to `ArmConstants`
            2. `kRobotDriveHight` move to `DriveConstants`