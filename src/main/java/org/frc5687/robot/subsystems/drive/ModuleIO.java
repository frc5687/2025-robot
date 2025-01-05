package org.frc5687.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {
    @AutoLog
    class ModuleIOInputs {
        public double driveVelocityRadsPerSec = 0.0;
        public double drivePositionRads = 0.0;
        public Rotation2d turnAbsolutePosition = new Rotation2d();
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(ModuleIOInputs inputs) {}

    /** Run to drive velocity setpoint with feedforward */
    default void runDriveVelocitySetpoint(double velocityRadsPerSec, double feedForward) {}

    /** Run to turn position setpoint */
    default void runTurnPositionSetpoint(double angleRads) {}
}