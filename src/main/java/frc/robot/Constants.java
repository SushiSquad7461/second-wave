// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final boolean TUNING_MODE = true;
    public static final class kDrivetrain {
        public static final int FRONT_LEFT_ID = 15;
        public static final int FRONT_RIGHT_ID = 4;
        public static final int BACK_LEFT_ID = 16;
        public static final int BACK_RIGHT_ID = 3;
        public static final int CURRENT_LIMIT = 30;
        // If Gyro is upsidedown set to negative one
        public static final int GYRO_INVERSION = 1;
        public static final int WHEEL_DIAMTER = 6; // inches
        public static double kaVoltSecondsSquaredPerMeter = 0.094246;
        public static double kvVoltSecondsPerMeter = 0.79639;
        public static double ksVolts = 0.094246;
        public static DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(0.69);
        public static double kMaxAccelerationMetersPerSecondSquared = 1;
        public static double kMaxSpeedMetersPerSecond = 4.92126;
        public static double kPDriveVel = 0.00001;
        public static double kIDriveVel = 0;
        public static double kDDriveVel = 0;
        public static double kRamseteB = 2;
        public static double kRamseteZeta = 0.7;
    }
    public static final class kOI {
        public static final int DRIVE_CONTROLER = 0;
    }
}
