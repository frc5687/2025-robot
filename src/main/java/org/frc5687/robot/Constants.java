/* Team 5687 (C)2020-2022 */
package org.frc5687.robot;

import org.frc5687.robot.subsystems.drive.SwerveModule.ModuleConfiguration;
import org.frc5687.robot.util.drivers.OutliersTalon;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.math.util.Units;

public class Constants {
    public static final int TICKS_PER_UPDATE = 1; // This is for the smartdashboard. 1 means it will update at the rate of the robot code, 5 will update every 5th loop and so on.
    public static final double METRIC_FLUSH_PERIOD = 1;
    public static final double UPDATE_PERIOD = 0.02; // 20 ms
    public static final double EPSILON = 1e-9;

    public static class SwerveModule {
        public static final String CAN_BUS = "CANivore";
        public static final int NUM_MODULES = 4;

        public static final OutliersTalon.Configuration DRIVE_CONFIG = new OutliersTalon.Configuration();
        public static final OutliersTalon.Configuration STEER_CONFIG = new OutliersTalon.Configuration();

        public static final double WHEEL_RADIUS = 0.0508; //11/15/2024 test (meters)
        public static final double GEAR_RATIO_DRIVE = (54.0 / 14.0) * (18.0 / 34.0) * (45.0 / 16.0);
        public static final double GEAR_RATIO_STEER = (48.0 / 18.0) * (96.0 / 16.0);

        // this is the motor config for the swerve motors
        static {
            DRIVE_CONFIG.TIME_OUT = 0.5;

            DRIVE_CONFIG.NEUTRAL_MODE = NeutralModeValue.Brake;
            DRIVE_CONFIG.INVERTED = InvertedValue.CounterClockwise_Positive;

            DRIVE_CONFIG.MAX_VOLTAGE = 12.0;

            DRIVE_CONFIG.MAX_CURRENT = 120; // Max control request current
            DRIVE_CONFIG.CURRENT_DEADBAND = 0.1;
        }

        static {
            STEER_CONFIG.TIME_OUT = 0.5;

            STEER_CONFIG.NEUTRAL_MODE = NeutralModeValue.Brake;
            STEER_CONFIG.INVERTED = InvertedValue.Clockwise_Positive;

            STEER_CONFIG.MAX_VOLTAGE = 12.0;

            STEER_CONFIG.MAX_SUPPLY_CURRENT = 30; // if using a foc control request these dont do anything, modify
            STEER_CONFIG.MAX_STATOR_CURRENT = 120;

            STEER_CONFIG.ENABLE_SUPPLY_CURRENT_LIMIT = false;
            STEER_CONFIG.ENABLE_STATOR_CURRENT_LIMIT = false;
            STEER_CONFIG.CURRENT_DEADBAND = 0.1;
        }

        public static final OutliersTalon.ClosedLoopConfiguration DRIVE_CONTROLLER_CONFIG = new OutliersTalon.ClosedLoopConfiguration();

        static {
            DRIVE_CONTROLLER_CONFIG.kP = 10.0;
            DRIVE_CONTROLLER_CONFIG.kI = 0.0;
            DRIVE_CONTROLLER_CONFIG.kD = 0.0;
            DRIVE_CONTROLLER_CONFIG.kV = 0.15;
            DRIVE_CONTROLLER_CONFIG.kS = 3;
        }
        public static final OutliersTalon.ClosedLoopConfiguration STEER_CONTROLLER_CONFIG = new OutliersTalon.ClosedLoopConfiguration();

        static {
            STEER_CONTROLLER_CONFIG.SLOT = 0;
            STEER_CONTROLLER_CONFIG.kP = 10.0; // FIXME
            STEER_CONTROLLER_CONFIG.kI = 0.0;
            STEER_CONTROLLER_CONFIG.kD = 0.0;
            STEER_CONTROLLER_CONFIG.kV = 0.0;

            STEER_CONTROLLER_CONFIG.IS_CONTINUOUS = true;
        }
    }

    public static class FieldConstants {
        public static final double FIELD_LENGTH = 16.54175;
        public static final double FIELD_WIDTH = 8.0137;
    }

    /**
     * Coordinate System
     *
     * <p>
     * (X, Y): X is N or S, N is + Y is W or E, W is +
     *
     * <p>
     * NW (+,+) NE (+,-)
     *
     * <p>
     * SW (-,+) SE (-,-)
     *
     * <p>
     * We go counter-counter clockwise starting at NW of chassis:
     *
     * <p>
     * NW, SW, SE, NE
     *
     * <p>
     * Note: when robot is flipped over, this is clockwise.
     */
    public static class DriveTrain {
        public static final String CAN_BUS = "CANivore";
        public static final int NUM_MODULES = 4;
        public static final double ROBOT_WEIGHT = Units.lbsToKilograms(120.0);

        // Size of the wheelbase in meters
        public static final double WIDTH = 0.5842; // meters
        public static final double LENGTH = 0.5842; // meters
        // Distance of swerve modules from center of robot
        public static final double SWERVE_NS_POS = LENGTH / 2.0;
        public static final double SWERVE_WE_POS = WIDTH / 2.0;

        public static final double ROBOT_RADIUS = Math.sqrt(WIDTH * WIDTH + LENGTH * LENGTH) / 2.0;

        public static final double MAX_MPS = DCMotor.getKrakenX60Foc(1).withReduction(Constants.SwerveModule.GEAR_RATIO_DRIVE).freeSpeedRadPerSec * SwerveModule.WHEEL_RADIUS;

        public static final double MAX_ANG_VEL = 2.0 * Math.PI; // Max rotation rate of robot (rads/s)
        public static final double MAX_ANG_ACC = 2.0 * Math.PI; // Max angular acceleration of robot (rads/s^2)

        /*
         * How to find offsets:
         * 
         * 1) Open Phoenix Tuner
         * 2) Zero CanCoder
         * 3) Config Tab
         * 4) Refresh
         * 5) Use "magnet offset" as offset in code
         */

        public static final ModuleConfiguration SOUTH_EAST_CONFIG = new ModuleConfiguration();

        static {
            SOUTH_EAST_CONFIG.moduleName = "South East";
            SOUTH_EAST_CONFIG.canBus = CAN_BUS;
            SOUTH_EAST_CONFIG.position = new Translation2d(-SWERVE_NS_POS, -SWERVE_WE_POS); // -,-

            SOUTH_EAST_CONFIG.encoderOffset = 0.4208985;
        }

        public static final ModuleConfiguration NORTH_EAST_CONFIG = new ModuleConfiguration();

        static {
            NORTH_EAST_CONFIG.moduleName = "North East";
            NORTH_EAST_CONFIG.canBus = CAN_BUS;
            NORTH_EAST_CONFIG.position = new Translation2d(SWERVE_NS_POS, -SWERVE_WE_POS); // +,-

            NORTH_EAST_CONFIG.encoderOffset = 1.0966796875;
        }

        public static final ModuleConfiguration NORTH_WEST_CONFIG = new ModuleConfiguration();

        static {
            NORTH_WEST_CONFIG.moduleName = "North West";
            NORTH_WEST_CONFIG.canBus = CAN_BUS;
            NORTH_WEST_CONFIG.position = new Translation2d(SWERVE_NS_POS, SWERVE_WE_POS); // +,+

            NORTH_WEST_CONFIG.encoderOffset = -0.41650390625;
        }

        public static final ModuleConfiguration SOUTH_WEST_CONFIG = new ModuleConfiguration();

        static {
            SOUTH_WEST_CONFIG.moduleName = "South West";
            SOUTH_WEST_CONFIG.canBus = CAN_BUS;
            SOUTH_WEST_CONFIG.position = new Translation2d(-SWERVE_NS_POS, SWERVE_WE_POS); // -,+

            SOUTH_WEST_CONFIG.encoderOffset = 0.361875;
        }

        public static final double TRANSLATION_DEADBAND = 0.05; // Avoid unintentional joystick movement
        public static final double ROTATION_DEADBAND = 0.05; // Avoid unintentional joystick movement
        public static final long DISABLE_TIME = 100; // ms

        // Maximum rates of motion
        public static final double POLE_THRESHOLD = Units.degreesToRadians(5.0);

        // PID controller settings
        public static final double HEADING_kP = 10.0;
        public static final double HEADING_kI = 0.0;
        public static final double HEADING_kD = 0.7;

        public static final double SNAP_TOLERANCE = Units.degreesToRadians(1.5);
        public static final double TARGET_TOLERANCE = Units.degreesToRadians(1);

        // AutoAlignDriveController PID
        public static final double kP = 3.3;
        public static final double kI = 0.0;
        public static final double kD = 0.05;

        public static final double POSITION_TOLERANCE = 0.01;
        public static final double HEADING_TOLERANCE = 0.04; // rad

        public static final Translation2d LOCATIONS[] =  {
            Constants.DriveTrain.NORTH_WEST_CONFIG.position,
            Constants.DriveTrain.NORTH_EAST_CONFIG.position,
            Constants.DriveTrain.SOUTH_WEST_CONFIG.position,
            Constants.DriveTrain.SOUTH_EAST_CONFIG.position
        };

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            LOCATIONS
        );
    }

    public static class Vision {
        public static final double VISION_kP = 3.0;
        public static final double VISION_kI = 0.0;
        public static final double VISION_kD = 0.2;
        public static final double AMBIGUITY_TOLERANCE = 0.4;
    }

    public static class VisionConfig {
        public static double STATE_STD_DEV_X = 0.01;
        public static double STATE_STD_DEV_Y = 0.01;
        public static double STATE_STD_DEV_ANGLE = Units.degreesToRadians(0.5); // imu deviations lower number to trust
                                                                                // more

        // we can't change the odometry stddev easily,,,, just change the vision stddev
        // --xavier bradford 02/25/24
        public static class Auto {
            public static double VISION_STD_DEV_X = 0.35;
            public static double VISION_STD_DEV_Y = 0.35;
            public static double VISION_STD_DEV_ANGLE = Units.degreesToRadians(900); // imu deviations lower number to
                                                                                     // trust
        }

        public static class Teleop {
            public static double VISION_STD_DEV_X = 0.15;
            public static double VISION_STD_DEV_Y = 0.15;
            public static double VISION_STD_DEV_ANGLE = Units.degreesToRadians(900); // imu deviations lower number to
                                                                                     // trust
        }
    }

     public static class RobotState {
        public static final double VISION_TIMESTAMP_FUDGE = -0.00;

        public static double VISION_AIMING_TOLERANCE = Units.degreesToRadians(2);
    }
}