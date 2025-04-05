package org.frc5687.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Optional;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.RobotStateManager.RobotCoordinate;
import org.frc5687.robot.subsystems.OutliersSubsystem;
import org.frc5687.robot.util.COMSwerveSetpointGenerator;
import org.frc5687.robot.util.COMVelocityLimiter;
import org.frc5687.robot.util.TunableDouble;

public class DriveSubsystem extends OutliersSubsystem<DriveInputs, DriveOutputs> {
    private final RobotContainer _container;
    private final DriveIO _driveIO;
    @NotLogged private final SwerveDriveKinematics _kinematics;
    @NotLogged private final SwerveDriveOdometry _odometry;
    private final SysIdRoutine _sysId;
    @Logged private String _sysIdStateString = "";

    private TunableDouble p = new TunableDouble("Drive", "kP", 0);
    private TunableDouble i = new TunableDouble("Drive", "kI", 0);
    private TunableDouble d = new TunableDouble("Drive", "kD", 0);
    private TunableDouble s = new TunableDouble("Drive", "kS", 0);

    private final COMVelocityLimiter _comLimiter;
    private final boolean _enableCOMLimiter = true;

    private final Translation2d[] _moduleLocations;
    private final COMSwerveSetpointGenerator _setpointGenerator;
    private SwerveSetpoint _currentSetpoint;

    private final RobotConfig _robotConfig;

    public boolean rosieEnabled;
    public boolean slowModeEnabled;

    public DriveSubsystem(RobotContainer container, DriveIO io, Translation2d[] moduleLocations) {
        super(container, io, new DriveInputs(), new DriveOutputs());
        _container = container;
        _driveIO = io;
        _moduleLocations = moduleLocations;

        _kinematics = new SwerveDriveKinematics(moduleLocations);
        _odometry = new SwerveDriveOdometry(_kinematics, _inputs.yawPosition, _inputs.modulePositions);

        _sysId =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null, null, null, (state) -> _sysIdStateString = state.toString()),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

        _robotConfig =
                new RobotConfig(
                        Units.Kilograms.of(Constants.DriveTrain.ROBOT_WEIGHT),
                        Units.KilogramSquareMeters.of(8.085),
                        new ModuleConfig(
                                Constants.SwerveModule.WHEEL_RADIUS,
                                Constants.SwerveModule.MAX_LINEAR_SPEED,
                                Constants.SwerveModule.COEFFICIENT_OF_FRICTION,
                                DCMotor.getKrakenX60Foc(1).withReduction(Constants.SwerveModule.GEAR_RATIO_DRIVE),
                                Constants.SwerveModule.DRIVE_CURRENT_LIMIT,
                                1),
                        moduleLocations[0],
                        moduleLocations[1],
                        moduleLocations[2],
                        moduleLocations[3]);

        _comLimiter = new COMVelocityLimiter();

        _setpointGenerator =
                new COMSwerveSetpointGenerator(
                        _robotConfig,
                        Constants.Motors.getKrakenX44(1).freeSpeedRadPerSec
                                / Constants.SwerveModule.GEAR_RATIO_STEER,
                        container,
                        _comLimiter);

        _currentSetpoint =
                new SwerveSetpoint(
                        new ChassisSpeeds(),
                        _inputs.measuredStates,
                        DriveFeedforwards.zeros(_robotConfig.numModules));

        rosieEnabled = true;
        slowModeEnabled = false;

        configureAutoBuilder(_robotConfig);
    }

    @Override
    protected void processInputs() {
        _inputs.odometryPose = _odometry.update(_inputs.yawPosition, _inputs.modulePositions);
    }

    @Override
    protected void periodic(DriveInputs inputs, DriveOutputs outputs) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            log("alliance", alliance.get());
        } else {
            log("alliance", "empty");
        }

        // Update PID if needed
        if (p.hasChanged() || i.hasChanged() || d.hasChanged() || s.hasChanged()) {
            _io.setPID(p.get(), i.get(), d.get(), 0, s.get(), 0, 0);
        }

        if (_comLimiter != null) {
            log("ElevatorHeight", _comLimiter.getLastElevatorHeight(), Importance.CRITICAL);
            log("SpeedLimitFactor", _comLimiter.getLastSpeedFactor(), Importance.CRITICAL);
        }

        if (outputs.bypassSetpointGenerator) {
            var states = _kinematics.toSwerveModuleStates(outputs.desiredSpeeds);
            outputs.desiredStates = states;
        } else {
            updateSetpoint();
            outputs.desiredStates = _currentSetpoint.moduleStates();
        }
    }

    public void setDesiredChassisSpeeds(ChassisSpeeds speeds) {
        // if (_enableCOMLimiter) {
        //     ChassisSpeeds limitedSpeeds = _comLimiter.limitSpeeds(speeds);
        //     _outputs.desiredSpeeds = limitedSpeeds;
        // } else {
        _outputs.desiredSpeeds = speeds;
        _outputs.bypassSetpointGenerator = false;
        // }
    }

    public void setDesiredChassisSpeedsBypassSetpointGenerator(ChassisSpeeds speeds) {
        // if (_enableCOMLimiter) {
        //     ChassisSpeeds limitedSpeeds = _comLimiter.limitSpeeds(speeds);
        //     _outputs.desiredSpeeds = limitedSpeeds;
        // } else {
        _outputs.desiredSpeeds = speeds;
        _outputs.bypassSetpointGenerator = true;
        // }
    }

    private void updateSetpoint() {
        _currentSetpoint =
                _setpointGenerator.generateSetpoint(
                        _currentSetpoint, _outputs.desiredSpeeds, Constants.UPDATE_PERIOD);
    }

    public Rotation2d getHeading() {
        return _inputs.yawPosition;
    }

    public Rotation2d getPitch() {
        return _inputs.pitchPosition;
    }

    public Rotation2d getRoll() {
        return _inputs.rollPosition;
    }

    public void zeroIMU() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            _driveIO.setYaw(Rotation2d.fromDegrees(180));
        } else {
            _driveIO.setYaw(Rotation2d.fromDegrees(0));
        }
    }

    public double getAngularVelocityYaw() {
        return _inputs.yawVelocityRadPerSec;
    }

    public Pose2d getPose() {
        return RobotStateManager.getInstance().getPose(RobotCoordinate.ROBOT_BASE_SWERVE).toPose2d();
    }

    public ChassisSpeeds getMeasuredChassisSpeeds() {
        return _kinematics.toChassisSpeeds(_inputs.measuredStates);
    }

    public SwerveDriveKinematics getKinematics() {
        return _kinematics;
    }

    public void resetPose(Pose2d pose) {
        RobotStateManager.getInstance().resetEstimatedPose(pose);
    }

    public Translation2d[] getModuleLocations() {
        return _moduleLocations;
    }

    public SwerveModulePosition[] getModulePositions() {
        return _inputs.modulePositions;
    }

    public void runCharacterization(double output) {
        _driveIO.runCharacterization(output);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        log("state", _sysIdStateString);
        return run(() -> runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(_sysId.quasistatic(direction));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        log("state", _sysIdStateString);
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(_sysId.dynamic(direction));
    }

    private void configureAutoBuilder(RobotConfig config) {
        AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getMeasuredChassisSpeeds,
                this::setDesiredChassisSpeedsBypassSetpointGenerator,
                new PPHolonomicDriveController(
                        new PIDConstants(6.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
                config,
                () -> {
                    Optional<Alliance> alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == Alliance.Red;
                },
                this);
    }
}
