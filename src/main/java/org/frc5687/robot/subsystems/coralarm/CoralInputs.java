package org.frc5687.robot.subsystems.coralarm;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import org.frc5687.robot.util.BaseInputs;

@Logged
public class CoralInputs extends BaseInputs {
    @Logged(name = "Coral State", importance = Logged.Importance.CRITICAL)
    public CoralState currentState = CoralState.IDLE_WITH_CORAL;

    @Logged(name = "Arm Angle (rads)", importance = Logged.Importance.CRITICAL)
    public double angleRads = 0.0;

    @Logged(name = "Arm Angular Velocity (rad per sec)", importance = Logged.Importance.CRITICAL)
    public double angularVelocityRadPerSec = 0.0;

    @Logged(name = "Motor Current (A)", importance = Logged.Importance.DEBUG)
    public double motorCurrent = 0.0;

    @Logged(name = "Motor Torque (Nm)", importance = Logged.Importance.DEBUG)
    public double motorTorque = 0.0;

    @Logged(name = "Arm Torque (Nm)", importance = Logged.Importance.DEBUG)
    public double armTorque = 0.0;

    @Logged(name = "Arm Pose (m radps)", importance = Logged.Importance.DEBUG)
    public Pose3d pose = new Pose3d();
}