// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

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

        public static final InvertedValue ANGLE_MOTOR_INVERT = InvertedValue.Clockwise_Positive;
        public static final InvertedValue DRIVE_MOTOR_INVERT = InvertedValue.Clockwise_Positive;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
                public static final int DRIVE_MOTOR_ID = 4;
                public static final int ANGLE_MOTOR_ID = 3;
                public static final int CANCODER_ID = 10;
                public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(257);
                public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                                ANGLE_MOTOR_ID,
                                CANCODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
                public static final int DRIVE_MOTOR_ID = 2;
                public static final int ANGLE_MOTOR_ID = 1;
                public static final int CANCODER_ID = 9;
                public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(137);
                public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                                ANGLE_MOTOR_ID,
                                CANCODER_ID, ANGLE_OFFSET);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
                public static final int DRIVE_MOTOR_ID = 8;
                public static final int ANGLE_MOTOR_ID = 7;
                public static final int CANCODER_ID = 12;
                public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(55 + 180);
                public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                                ANGLE_MOTOR_ID,
                                CANCODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
                public static final int DRIVE_MOTOR_ID = 6;
                public static final int ANGLE_MOTOR_ID = 5;
                public static final int CANCODER_ID = 11;
                public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(50);
                public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                                ANGLE_MOTOR_ID,
                                CANCODER_ID, ANGLE_OFFSET);
        }

        /* Chosen Module */
        public static final COTSFalconSwerveConstants chosenModule = COTSFalconSwerveConstants
                        .SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Gyro reversed */
        public static final boolean INVERT_GYRO = false;

        public static final SensorDirectionValue CANCODER_INVERT = SensorDirectionValue.CounterClockwise_Positive;

        /* Angle Motor PID Values */
        public static final double AZIMUTH_P = chosenModule.angleKP;
        public static final double AZIMUTH_I = chosenModule.angleKI;
        public static final double AZIMUTH_D = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double DRIVE_P = 0.1;
        public static final double DRIVE_I = 0.0;
        public static final double DRIVE_D = 0.0;
        public static final double DRIVE_F = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double DRIVE_S = 0.32;
        public static final double DRIVE_V = 1.51;
        public static final double DRIVE_A = 0.27;

        public static final double RATE_LIMITER = 1.5;

        /* Swerve Current Limiting */
        public static final int AZIMUTH_CURRENT_LIMIT = 25;
        public static final int AZIMUTH_CURRENT_THRESHOLD = 40;
        public static final double AZIMUTH_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean AZIMUTH_ENABLE_CURRENT_LIMIT = true;

        public static final int DRIVE_CURRENT_LIMIT = 40;
        public static final int DRIVE_CURRENT_THRESHOLD = 60;
        public static final double DRIVE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Neutral Modes */
        public static final NeutralModeValue AZIMUTH_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;

        /* Swerve Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = chosenModule.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = chosenModule.angleGearRatio;

        /* Swerve Profiling Values */
        public static final double MAX_ANGULAR_VELOCITY = Math.PI * 4.12 * 0.5; // radians per second (theoretical
                                                                                // calculation)
        public static final double TURN_IN_PLACE_SPEED = 0.5;
        public static final double A_RATE_LIMITER = 2.0; // Slew Rate Limiter Constant

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
                        Math.PI, (Math.PI * Math.PI));

        public static final double DRIVETRAIN_WIDTH = Units.inchesToMeters(21.75);
        public static final double DRIVETRAIN_LENGTH = Units.inchesToMeters(21.75);
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
        public static final double WHEEL_CIRCUMFERENCE = chosenModule.wheelCircumference;

        public static final double DRIVEBASE_RADIUS = Units
                        .inchesToMeters(14.67247);

        /* Swerve Kinematics */
        public static final Translation2d[] moduleTranslations = new Translation2d[] {
                        new Translation2d(DRIVETRAIN_LENGTH / 2.0, DRIVETRAIN_WIDTH / 2.0),
                        new Translation2d(DRIVETRAIN_LENGTH / 2.0, -DRIVETRAIN_WIDTH / 2.0),
                        new Translation2d(-DRIVETRAIN_LENGTH / 2.0, DRIVETRAIN_WIDTH / 2.0),
                        new Translation2d(-DRIVETRAIN_LENGTH / 2.0, -DRIVETRAIN_WIDTH / 2.0) };

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(moduleTranslations);

        public static final double STICK_DEADBAND = 0.1;
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;

        public static final double MAX_SPEED = Units.feetToMeters(17.1);
        public static final double AUTO_MAX_ACCELERATION_MPS_SQUARED = 3;


        // LimeLight Constants, not actually used in code, but used in Limelight Configs

        /*
         * Front LL:
         * Forward: .1016
         * Right: -.2921
         * Up: .2819
         * 
         * Rear LL:
         * 
         * 
         */
        // values from Team Spectrum 3847’s X-Ray robot from last year
        // https://www.chiefdelphi.com/t/swerve-drive-pose-estimator-and-add-vision-measurement-using-limelight-is-very-jittery/453306/5
        public static final Vector<N3> STATE_STDS = VecBuilder.fill(0.1, 0.1, 10);

        /**
         * Standard deviations of the vision measurements. Increase these numbers to
         * trust global measurements from vision less. This matrix is in the form
         * [x, y, theta]ᵀ, with units in meters and radians.
         */

        // values from Team Spectrum 3847’s X-Ray robot from last year
        public static final Vector<N3> VISION_STDS = VecBuilder.fill(5, 5, 500);

}
