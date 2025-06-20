package org.frc5687.robot.util;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.subsystems.elevator.ElevatorState;

public class COMSwerveSetpointGenerator {
    private final SwerveSetpointGenerator _baseGenerator;
    private final COMVelocityLimiter _comLimiter;
    private final RobotContainer _container;

    private final TunableDouble _accelScalingMultiplier =
            new TunableDouble("COMSetpoint", "AccelScalingMultiplier", 0.8);
    private final TunableDouble _angularVelScalingMultiplier =
            new TunableDouble("COMSetpoint", "AngularVelScalingMultiplier", 2.0);
    private final TunableDouble _angularAccelScalingMultiplier =
            new TunableDouble("COMSetpoint", "AngularAccelScalingMultiplier", 0.7);

    public COMSwerveSetpointGenerator(
            RobotConfig config,
            double maxSteerVelocityRadsPerSec,
            RobotContainer container,
            COMVelocityLimiter comLimiter) {
        _baseGenerator = new SwerveSetpointGenerator(config, maxSteerVelocityRadsPerSec);
        _container = container;
        _comLimiter = comLimiter;
    }

    public SwerveSetpoint generateSetpoint(
            SwerveSetpoint prevSetpoint, ChassisSpeeds desiredSpeeds, double dt) {

        // only limit if we are over L2
        if (_container.getElevator().getElevatorHeight() < ElevatorState.L2_CORAL_PLACING.getHeight()) {
            return _baseGenerator.generateSetpoint(prevSetpoint, desiredSpeeds, dt);
        }

        PathConstraints heightConstraints = createHeightConstraints(_comLimiter.calculateSpeedFactor());
        return _baseGenerator.generateSetpoint(prevSetpoint, desiredSpeeds, heightConstraints, dt);
    }

    private PathConstraints createHeightConstraints(double speedFactor) {
        double baseMaxVel = Constants.DriveTrain.MAX_MPS;
        double baseMaxAccel = baseMaxVel * 3.0;
        double baseMaxAngVel = Constants.DriveTrain.MAX_ANG_VEL;
        double baseMaxAngAccel = Constants.DriveTrain.MAX_ANG_ACC;

        double accelScaling = _accelScalingMultiplier.get();
        double angularVelScaling = _angularVelScalingMultiplier.get();
        double angularAccelScaling = _angularAccelScalingMultiplier.get();

        double accelFactor = speedFactor * accelScaling;
        double angVelFactor = speedFactor * angularVelScaling;
        double angAccelFactor = speedFactor * angularAccelScaling;

        double adjustedMaxVel = baseMaxVel * speedFactor;
        double adjustedMaxAccel = baseMaxAccel * accelFactor;
        double adjustedMaxAngVel = baseMaxAngVel * angVelFactor;
        double adjustedMaxAngAccel = baseMaxAngAccel * angAccelFactor;

        adjustedMaxVel = Math.max(0.5, adjustedMaxVel);
        adjustedMaxAccel = Math.max(1.0, adjustedMaxAccel);
        adjustedMaxAngVel = Math.max(0.5, adjustedMaxAngVel);
        adjustedMaxAngAccel = Math.max(1.0, adjustedMaxAngAccel);

        return new PathConstraints(
                adjustedMaxVel, adjustedMaxAccel, adjustedMaxAngVel, adjustedMaxAngAccel);
    }
}
