/* Team 5687 (C)2020-2022 */
package org.frc5687.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.frc5687.robot.subsystems.drive.modules.SwerveModuleConfig;
import org.frc5687.robot.subsystems.intake.IntakeConfig;
import org.frc5687.robot.util.PIDConstants;

// import edu.wpi.first.math.util.Units;

public class Constants {
    public static final int TICKS_PER_UPDATE =
            1; // This is for the smartdashboard. 1 means it will update at the rate of the robot code, 5
    // will update every 5th loop and so on.
    public static final double METRIC_FLUSH_PERIOD = 1;
    public static final double UPDATE_PERIOD = 0.02; // 20 ms
    public static final double EPSILON = 1e-9;

    public static class SwerveModule {
        public static final String CAN_BUS = "CANivore";
        public static final int NUM_MODULES = 4;

        // Physical Configuration
        public static final double WHEEL_RADIUS = 0.0508; // 11/15/2024 test (meters)
        public static final double GEAR_RATIO_DRIVE = (54.0 / 14.0) * (18.0 / 34.0) * (45.0 / 16.0);
        public static final double GEAR_RATIO_STEER = (48.0 / 18.0) * (96.0 / 16.0);

        // Drive Motor Configuration
        public static final PIDConstants DRIVE_PID = new PIDConstants(0.01, 0.0, 0.0); // 10.0 kp
        public static final double DRIVE_KS =
                0.032292; // 3 // This is the voltage to overcome static friction taken from sysid 0.0032292
        public static final double DRIVE_KV = 0.15; // 0.15 taken from sysid 2.24
        public static final double DRIVE_KA = 0; // sysid 0.131

        // public static final PIDConstants DRIVE_PID = new PIDConstants(0.0, 0.0, 0.0); //10.0 kp
        // public static final double DRIVE_KS = 0.0; //3 // This is the voltage to overcome static
        // friction taken from sysid 0.0032292
        // public static final double DRIVE_KV = 0; //0.15 taken from sysid 2.24
        // public static final double DRIVE_KA = 0; // sysid 0.131
        public static final double DRIVE_CURRENT_LIMIT = 80.0;

        public static final PIDConstants STEER_PID = new PIDConstants(30, 0.0, 0.0); // 10 kp
        public static final double STEER_KS = 0.0;
        public static final double STEER_KV = 0.0;
        public static final double STEER_KA = 0.0;
        public static final double STEER_CURRENT_LIMIT = 30.0;

        public static final double STEER_MOTION_CRUISE_VELOCITY = 100.0; // rad/s
        public static final double STEER_MOTION_ACCELERATION = 200.0; // rad/s^2

        public static final double COUPLING_RATIO = 0.0;

        public static final double MAX_LINEAR_SPEED = 5.0; // meters per second
        public static final double MAX_ANGULAR_SPEED = Math.PI * 2.0; // radians per second

        public static SwerveModuleConfig createModuleConfig(
                String name, double offset, boolean driveInverted, boolean steerInverted) {
            return new SwerveModuleConfig(
                    name,
                    WHEEL_RADIUS,
                    GEAR_RATIO_DRIVE,
                    GEAR_RATIO_STEER,
                    COUPLING_RATIO,
                    DRIVE_PID,
                    DRIVE_KS,
                    DRIVE_KV,
                    DRIVE_KA,
                    DRIVE_CURRENT_LIMIT,
                    STEER_PID,
                    STEER_KS,
                    STEER_KV,
                    STEER_KA,
                    STEER_CURRENT_LIMIT,
                    STEER_MOTION_CRUISE_VELOCITY,
                    STEER_MOTION_ACCELERATION,
                    offset,
                    driveInverted,
                    steerInverted);
        }
    }

    public static class FieldConstants {
        public static final double FIELD_LENGTH = 16.54175;
        public static final double FIELD_WIDTH = 8.0137;
    }

    /**
     * Coordinate System
     *
     * <p>(X, Y): X is N or S, N is + Y is W or E, W is +
     *
     * <p>NW (+,+) NE (+,-)
     *
     * <p>SW (-,+) SE (-,-)
     *
     * <p>We go counter-counter clockwise starting at NW of chassis:
     *
     * <p>NW, SW, SE, NE
     *
     * <p>Note: when robot is flipped over, his is clockwise.
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

        public static final double MAX_MPS =
                DCMotor.getKrakenX60Foc(1)
                                .withReduction(Constants.SwerveModule.GEAR_RATIO_DRIVE)
                                .freeSpeedRadPerSec
                        * SwerveModule.WHEEL_RADIUS;

        public static final double MAX_ANG_VEL = 2.0 * Math.PI; // Max rotation rate of robot (rads/s)
        public static final double MAX_ANG_ACC =
                2.0 * Math.PI; // Max angular acceleration of robot (rads/s^2)

        /*
         * How to find offsets:
         *
         * 1) Open Phoenix Tuner
         * 2) Zero CanCoder
         * 3) Config Tab
         * 4) Refresh
         * 5) Use "magnet offset" as offset in code
         *
         */
        public static final SwerveModuleConfig NW_CONFIG =
                SwerveModule.createModuleConfig("NW", -0.41650390625, false, false);
        public static final SwerveModuleConfig NE_CONFIG =
                SwerveModule.createModuleConfig("NE", 1.0966796875, false, false);
        public static final SwerveModuleConfig SW_CONFIG =
                SwerveModule.createModuleConfig("SW", 0.361875, false, false);
        public static final SwerveModuleConfig SE_CONFIG =
                SwerveModule.createModuleConfig("SE", 0.4208985, false, false);

        public static final SwerveModuleConfig[] MODULE_CONFIGS = {
            NW_CONFIG, NE_CONFIG, SW_CONFIG, SE_CONFIG
        };

        public static final Translation2d[] MODULE_LOCATIONS = {
            new Translation2d(SWERVE_NS_POS, SWERVE_WE_POS), // NW (front left)  [+,+]
            new Translation2d(SWERVE_NS_POS, -SWERVE_WE_POS), // NE (front right) [+,-]
            new Translation2d(-SWERVE_NS_POS, SWERVE_WE_POS), // SW (back left)   [-,+]
            new Translation2d(-SWERVE_NS_POS, -SWERVE_WE_POS) // SE (back right)  [-,-]
        };

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

        // public static final Translation2d LOCATIONS[] =  {
        //     Constants.DriveTrain.NORTH_WEST_CONFIG.position,
        //     Constants.DriveTrain.NORTH_EAST_CONFIG.position,
        //     Constants.DriveTrain.SOUTH_WEST_CONFIG.position,
        //     Constants.DriveTrain.SOUTH_EAST_CONFIG.position
        // };

        // public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        //     LOCATIONS
        // );
    }

    public static class Intake {

        public static final String CAN_BUS = "CANivore";

        public static final boolean PIVOT_INVERTED = true;
        public static final boolean INTAKE_INVERTED = true;
        public static final boolean ROLLER_INVERTED = true;

        public static final double CURRENT_LIMIT = 60;

        public static final double kP = 7.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;

    }

    public static class Elevator {

        public static final double PERIOD = 0.004;
        public static final String CANBUS = "CANivore";

        public static final int NUM_MOTORS = 1;
        public static final DCMotor MOTOR = DCMotor.getKrakenX60Foc(NUM_MOTORS);
        public static final double GEAR_RATIO = 9.3;
        public static final double MASS = 13.6078; // kg
        public static final double DRUM_RADIUS = Units.inchesToMeters(2.25 / 2); // m
        public static final double MIN_HEIGHT = 0.0; // m
        public static final double MAX_HEIGHT = 0.7295515; // m THIS IS THE HEIGHT OF THE FIRST STAGE
        public static final double EFFICIENCY = 0.85;
        public static final double MAX_VELOCITY_MPS =
                (MOTOR.freeSpeedRadPerSec / GEAR_RATIO) * (2 * Math.PI * DRUM_RADIUS);
        public static final double MAX_ACCELERATION_MPSS = 50;
        public static final double MAX_JERK_MPSSS = 400;

        public static final double HOLD_kP = 2.5; // 2.5
        public static final double HOLD_kI = 0.0;
        public static final double HOLD_kD = 0.025;
        public static final double HOLD_kS = 0.0;
        public static final double HOLD_kV = 0.0;
        public static final double HOLD_kA = 0.0;

        public static final double MOTION_kP = 7.5;
        public static final double MOTION_kI = 0.0;
        public static final double MOTION_kD = 0.0;
        public static final double MOTION_kS = 0.0;
        public static final double MOTION_kV = 0.0;
        public static final double MOTION_kA = 0.0;

        public static final double MOTION_EXPO_kP = 0.0;
        public static final double MOTION_EXPO_kI = 0.0;
        public static final double MOTION_EXPO_kD = 0.0;
        public static final double MOTION_EXPO_kS = 0.0;
        public static final double MOTION_EXPO_kV = 0.0;
        public static final double MOTION_EXPO_kA = 0.0;
        public static final double MOTION_EXPO_kG = 0.0;

        public static final double PITCH_kP = 0.9;
        public static final double PITCH_kD = 0.09;
        public static final double ROLL_kP = 0.1;
        public static final double ROLL_kD = 0.01;

        public static final boolean NORTH_WEST_INVERTED = true;
        public static final boolean NORTH_EAST_INVERTED = false;
        public static final boolean SOUTH_EAST_INVERTED = false;

        public static final double CURRENT_LIMIT = 60.0;

        public static final double MAX_POSITION_CORRECTION = 0.02;
        public static final double VELOCITY_TOLERANCE = 0.02;
    }

    public static class AlgaeArm {
        public static final PIDConstants SIM_PID_CONSTANTS = new PIDConstants(10, 0, 0);

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final int NUM_MOTORS = 1;

        public static final DCMotor GEARBOX = DCMotor.getBag(1);
        public static final double GEAR_RATIO = 300;
        public static final double ARM_LENGTH = 0.589;
        public static final double ARM_MASS = Units.lbsToKilograms(10);
        public static final double MOI_ARM = SingleJointedArmSim.estimateMOI(ARM_LENGTH, ARM_MASS);
        public static final double MIN_ANGLE = 0.0;
        public static final double MAX_ANGLE = Units.degreesToRadians(270.0);

        public static final double MAX_VELOCITY_RAD_PER_SEC = GEARBOX.freeSpeedRadPerSec / GEAR_RATIO;
        public static final double MAX_ACCELERATION_RAD_PER_SEC_SQUARED = 2.0 * Math.PI;
    }

    public static class CoralArm {

        public static final PIDConstants SIM_PID_CONSTANTS = new PIDConstants(10, 0, 0);

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final int NUM_MOTORS = 1;

        public static final DCMotor GEARBOX = DCMotor.getBag(1);
        public static final double GEAR_RATIO = 300;
        public static final double ARM_LENGTH = Units.inchesToMeters(12.0);
        public static final double ARM_MASS = Units.lbsToKilograms(10);
        public static final double MOI_ARM = SingleJointedArmSim.estimateMOI(ARM_LENGTH, ARM_MASS);
        public static final double MIN_ANGLE = 0.0;
        public static final double MAX_ANGLE = Units.degreesToRadians(180);

        public static final double MAX_VELOCITY_RAD_PER_SEC = GEARBOX.freeSpeedRadPerSec / GEAR_RATIO;
        public static final double MAX_ACCELERATION_RAD_PER_SEC_SQUARED = 2.0 * Math.PI;
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
        public static double STATE_STD_DEV_ANGLE =
                Units.degreesToRadians(0.5); // imu deviations lower number to trust

        // more

        // we can't change the odometry stddev easily,,,, just change the vision stddev
        // --xavier bradford 02/25/24
        public static class Auto {
            public static double VISION_STD_DEV_X = 0.35;
            public static double VISION_STD_DEV_Y = 0.35;
            public static double VISION_STD_DEV_ANGLE =
                    Units.degreesToRadians(900); // imu deviations lower number to
            // trust
        }

        public static class Teleop {
            public static double VISION_STD_DEV_X = 0.15;
            public static double VISION_STD_DEV_Y = 0.15;
            public static double VISION_STD_DEV_ANGLE =
                    Units.degreesToRadians(900); // imu deviations lower number to
            // trust
        }
    }

    public static class RobotState {
        public static final double VISION_TIMESTAMP_FUDGE = -0.00;

        public static double VISION_AIMING_TOLERANCE = Units.degreesToRadians(2);
    }
}
