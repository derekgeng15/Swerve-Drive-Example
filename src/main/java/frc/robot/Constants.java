// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper.GearRatio;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class kChassis{
        public static final double kTrackWidth = 30;
        public static final GearRatio kGearRatio = Mk4iSwerveModuleHelper.GearRatio.L1;
        public static final double kMaxMetersPerSecond = 14;
        public static final double kMaxRadiansPerSecond = 14;
    }

    public static final class kCANID{
        public static final int kFrontLeftDrive = 1;
        public static final int kFrontRightDrive = 2;
        public static final int kBackLeftDrive = 3;
        public static final int kBackRightDrive = 4;

        public static final int kFrontLeftSteer = 5;
        public static final int kFrontRightSteer = 6;
        public static final int kBackLeftSteer = 7;
        public static final int kBackRightSteer = 8;

        public static final int kFrontLeftCANCoder = 9;
        public static final int kFrontRightCANCoder = 10;
        public static final int kBackLeftCANCoder = 11;
        public static final int kBackRightCANCoder = 12;
    }
    
    public static final class OI{
        public static final int kXboxController = 1;
    }
}
