// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public final class Constants {

    public static final class DriveBase {
        public static final double MAX_VOLTAGE = 12.0;

        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5779;//0.71;
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5779;


        public static final double xRateLimit = 5;
        public static final double yRateLimit = 5;
        public static final double rotRateLimit = 2;

        public static final int driveCurrentLimit = 25;
        public static final int rotCurrentLimit = 25;

    }

    public static final class Modules {
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 3;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 13;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 1;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(266.6);

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 12;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 11;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 2;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(68.7);

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 14;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 4;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(160.83);

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 2;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 6;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 3;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(265.5);
    }

    public static final class Auto {
        public static final double kS = 0.18656;
        public static final double kV = 2.64;
        public static final double kA = 0.38134;
        public static final double kP = 0; //3.4557;
    }

}
