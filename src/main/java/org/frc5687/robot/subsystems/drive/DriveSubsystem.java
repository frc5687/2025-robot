package org.frc5687.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;

import org.frc5687.robot.Constants;
import org.frc5687.robot.subsystems.OutliersSubsystem;
import org.frc5687.robot.subsystems.SubsystemIO;
import org.frc5687.robot.subsystems.drive.modules.SwerveModule;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class DriveSubsystem extends OutliersSubsystem<DriveInputs, DriveOutputs> {
    @NotLogged
    private final SwerveModule[] _modules;
    @NotLogged
    private final SwerveDriveKinematics _kinematics;
    @NotLogged
    private final SwerveDriveOdometry _odometry;
    private final SysIdRoutine _sysId;
    @Logged
    private String _sysIdStateString = "";

    @Logged
    private ChassisSpeeds _desiredChassisSpeeds = new ChassisSpeeds();

    public DriveSubsystem(
            SubsystemIO<DriveInputs, DriveOutputs> io,
            SwerveModule[] modules,
            Translation2d[] moduleLocations) {
        super(io, new DriveInputs(), new DriveOutputs());
        _modules = modules;
        _kinematics = new SwerveDriveKinematics(moduleLocations);
        _odometry = new SwerveDriveOdometry(
                _kinematics,
                _inputs.yawPosition,
                _inputs.modulePositions);
        _sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        null,
                        null,
                        (state) -> _sysIdStateString = state.toString()),
                new SysIdRoutine.Mechanism(
                        (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    }

    @Override
    protected void processInputs() {
        // Update module states
        for (int i = 0; i < 4; i++) {
            _inputs.measuredStates[i] = _modules[i].getState();
            _inputs.modulePositions[i] = _modules[i].getPosition();
        }

        // Update odometry
        _odometry.update(_inputs.yawPosition, _inputs.modulePositions);
    }

    @Override
    protected void periodic(DriveInputs inputs, DriveOutputs outputs) {
        outputs.desiredStates = _kinematics.toSwerveModuleStates(_desiredChassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(
                outputs.desiredStates,
                Constants.DriveTrain.MAX_MPS);

        for (int i = 0; i < 4; i++) {
            _modules[i].setDesiredState(outputs.desiredStates[i]);
        }
    }

    public void setDesiredChassisSpeeds(ChassisSpeeds speeds) {
        _desiredChassisSpeeds = speeds;
    }

    public Rotation2d getHeading() {
        return _inputs.yawPosition;
    }

    public Pose2d getPose() {
        return _odometry.getPoseMeters();
    }

    public ChassisSpeeds getMeasuredChassisSpeeds() {
        return _kinematics.toChassisSpeeds(_inputs.measuredStates);
    }

    public SwerveDriveKinematics getKinematics() {
        return _kinematics;
    }

    public void resetPose(Pose2d pose) {
        _odometry.resetPosition(_inputs.yawPosition, _inputs.modulePositions, pose);
    }

    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            _modules[i].runCharacterization(output);
        }
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(_sysId.quasistatic(direction));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(_sysId.dynamic(direction));
    }
}