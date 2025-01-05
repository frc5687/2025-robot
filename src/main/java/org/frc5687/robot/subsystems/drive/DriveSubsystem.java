package org.frc5687.robot.subsystems.drive;

import org.frc5687.robot.Constants;
import org.frc5687.robot.subsystems.drive.DriveIO.DriveIOInputs;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.console.ConsoleSource;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.Units;
import java.util.Optional;

public class DriveSubsystem extends SubsystemBase {
    private final DriveIO _io;
    private final DriveIOInputs _inputs = new DriveIOInputs();
    
    private final SwerveDriveKinematics _kinematics;
    private final SwerveSetpointGenerator _setpointGenerator;

    private SwerveSetpoint _currentSetpoint = new SwerveSetpoint(
        new ChassisSpeeds(), new SwerveModuleState[4], DriveFeedforwards.zeros(4));
    private ChassisSpeeds _desiredChassisSpeeds = new ChassisSpeeds();

    public DriveSubsystem(DriveIO io) {
        _io = io;

        // I dont really like this, maybe move kinematics to DriveIO?

        Translation2d moduleLocations[] =  {
            Constants.DriveTrain.NORTH_WEST_CONFIG.position,
            Constants.DriveTrain.NORTH_EAST_CONFIG.position,
            Constants.DriveTrain.SOUTH_WEST_CONFIG.position,
            Constants.DriveTrain.SOUTH_EAST_CONFIG.position
        };

        _kinematics = new SwerveDriveKinematics(moduleLocations);
        
        // var motor = DCMotor.getKrakenX60(1).withReduction(Constants.SwerveModule.GEAR_RATIO_DRIVE);
        var config = new RobotConfig(
            Units.Pounds.of(50.0),
            Units.KilogramSquareMeters.of(9),
            new ModuleConfig(
                Constants.SwerveModule.WHEEL_RADIUS, // wheel radius in meters
                Constants.DriveTrain.MAX_MPS, // max module speed in m/s
                1.0, // drive base radius in meters
                DCMotor.getKrakenX60Foc(1).withReduction(Constants.SwerveModule.GEAR_RATIO_DRIVE), // drive motor and reduction
                120, // drive current limit
                1 
            ),
            moduleLocations[0], // NW 
            moduleLocations[1], // NE 
            moduleLocations[2], // SW
            moduleLocations[3]  // SE
        );
        
        _setpointGenerator = new SwerveSetpointGenerator(
                config,
                DCMotor.getKrakenX60(1).freeSpeedRadPerSec / (Constants.SwerveModule.GEAR_RATIO_STEER) // steer motor max speed divided by reduction
        );
        _currentSetpoint = new SwerveSetpoint(_desiredChassisSpeeds, _inputs.measuredStates, DriveFeedforwards.zeros(config.numModules));
        
        configureAutoBuilder(config);
    }

    @Override
    public void periodic() {
        _io.updateInputs(_inputs);
        updateModuleStatesSimple();
        //updateDesiredStates();
        //_io.setModuleStates(_currentSetpoint.moduleStates());
    }

    public void setVelocity(ChassisSpeeds speeds) {
        Logger.recordOutput("Requested Speeds", speeds);
        _desiredChassisSpeeds = speeds;
    }

    public void updateModuleStatesSimple() {
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(_desiredChassisSpeeds, 0.02);
        SwerveModuleState[] setpointStates = _kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, Constants.DriveTrain.MAX_MPS * 0.8);
        var moduleStates = _kinematics.toSwerveModuleStates(_desiredChassisSpeeds);
        _io.setModuleStates(moduleStates);
    }

    private void updateDesiredStates() {
        Pose2d robotPoseVel = new Pose2d(
            _desiredChassisSpeeds.vxMetersPerSecond * Constants.UPDATE_PERIOD,
            _desiredChassisSpeeds.vyMetersPerSecond * Constants.UPDATE_PERIOD,
            Rotation2d.fromRadians(_desiredChassisSpeeds.omegaRadiansPerSecond * Constants.UPDATE_PERIOD));

        Twist2d twistVel = new Pose2d().log(robotPoseVel);
        ChassisSpeeds updatedSpeeds = new ChassisSpeeds(
            twistVel.dx / Constants.UPDATE_PERIOD,
            twistVel.dy / Constants.UPDATE_PERIOD,
            twistVel.dtheta / Constants.UPDATE_PERIOD);
        _currentSetpoint = _setpointGenerator.generateSetpoint(_currentSetpoint, updatedSpeeds, Constants.UPDATE_PERIOD);
    }

    private void configureAutoBuilder(RobotConfig config) {
        AutoBuilder.configure(
            () -> new Pose2d(), // Robot pose supplier
            (pose) -> {}, // Method to reset odometry
            this::getMeasuredChassisSpeeds,
            this::setVelocity,
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0),
                new PIDConstants(5.0, 0.0, 0.0)),
            config,
            () -> {
                Optional<Alliance> alliance = DriverStation.getAlliance();
                return alliance.isPresent() && alliance.get() == Alliance.Red;
            },
            this);
    }

    public Rotation2d getHeading() {
        return _io.getHeading();
    }

    public ChassisSpeeds getMeasuredChassisSpeeds() {
        return _kinematics.toChassisSpeeds(_inputs.measuredStates);
    }

    public void zeroGyroscope() {
        _io.zeroGyroscope();
    }

    public DriveIOInputs getDriveIOInputs() {
        return _inputs;
    }
}