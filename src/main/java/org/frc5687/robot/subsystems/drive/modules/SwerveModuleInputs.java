package org.frc5687.robot.subsystems.drive.modules;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import org.frc5687.robot.util.BaseInputs;

@Logged
public class SwerveModuleInputs extends BaseInputs {
    // Drive motor state
    @Logged(name = "Drive/Position (m)", importance = Logged.Importance.DEBUG)
    public double drivePositionMeters = 0.0;

    @Logged(name = "Drive/Velocity (mps)", importance = Logged.Importance.DEBUG)
    public double driveVelocityMPS = 0.0;

    @Logged(name = "Drive/Applied Voltage (V)", importance = Logged.Importance.DEBUG)
    public double driveAppliedVolts = 0.0;

    @Logged(name = "Drive/Current (A)", importance = Logged.Importance.DEBUG)
    public double driveCurrentAmps = 0.0;

    @Logged(name = "Drive/Temperature (C)", importance = Logged.Importance.DEBUG)
    public double driveTempCelsius = 0.0;

    // Steer motor state
    @Logged(name = "Steer/Position (rad)", importance = Logged.Importance.CRITICAL)
    public Rotation2d steerAngle = new Rotation2d();

    @Logged(name = "Steer/Velocity (radps)", importance = Logged.Importance.DEBUG)
    public double steerVelocityRadPerSec = 0.0;

    @Logged(name = "Steer/Applied Voltage (V)", importance = Logged.Importance.DEBUG)
    public double steerAppliedVolts = 0.0;

    @Logged(name = "Steer/Current (A)", importance = Logged.Importance.DEBUG)
    public double steerCurrentAmps = 0.0;

    @Logged(name = "Steer/Temperature (C)", importance = Logged.Importance.DEBUG)
    public double steerTempCelsius = 0.0;

    // Absolute encoder state
    @Logged(name = "Absolute/Angle (rad)", importance = Logged.Importance.CRITICAL)
    public Rotation2d absoluteAngle = new Rotation2d();
}
