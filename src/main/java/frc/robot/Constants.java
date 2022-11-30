// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveBase {
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5779;//0.71;
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5779;
        public static final double xRateLimit = 0.1;
        public static final double yRateLimit = 0.1;
        public static final double rotRateLimit = 0.1;
    }

    public static final class Auto {
        public static final double kS = 0.18656;
        public static final double kV = 2.64;
        public static final double kA = 0.38134;
        public static final double kP = 0;//3.4557;
    }

    public static final class Modules {
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 3;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 13;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 1;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(266.6); // FIXME Measure and set front left steer offset

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 12;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 11;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 2;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(68.7); // FIXME Measure and set front right steer offset

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 14;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 4;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(160.83); // FIXME Measure and set back left steer offset

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 2;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 6;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 3;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(265.5); // FIXME Measure and set back right steer offset
    }

}
