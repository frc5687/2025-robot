package org.frc5687.robot.subsystems.drive.modules;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import org.frc5687.robot.util.BaseOutputs;

@Logged
public class SwerveModuleOutputs extends BaseOutputs {
    // Drive motor commands
    @Logged(name = "Drive/Voltage Command (V)", importance = Logged.Importance.DEBUG)
    public double driveVoltage = 0.0;

    @Logged(name = "Drive/Velocity Setpoint (mps)", importance = Logged.Importance.DEBUG)
    public double driveVelocitySetpointMPS = 0.0;

    @Logged(name = "Drive/Control Mode", importance = Logged.Importance.DEBUG)
    public ModuleControlMode driveControlMode = ModuleControlMode.VOLTAGE;

    // Steer motor commands
    @Logged(name = "Steer/Angle Setpoint (rad)", importance = Logged.Importance.DEBUG)
    public Rotation2d steerAngleSetpoint = new Rotation2d();

    @Logged(name = "Steer/Voltage Command (V)", importance = Logged.Importance.DEBUG)
    public double steerVoltage = 0.0;

    @Logged(name = "Steer/Control Mode", importance = Logged.Importance.DEBUG)
    public ModuleControlMode steerControlMode = ModuleControlMode.VOLTAGE;

    // If using feedforward
    @Logged(name = "Drive/FF Voltage (V)", importance = Logged.Importance.DEBUG)
    public double driveFeedforwardVolts = 0.0;

    @Logged(name = "Steer/FF Voltage (V)", importance = Logged.Importance.DEBUG)
    public double steerFeedforwardVolts = 0.0;
}
