package org.frc5687.robot.subsystems.elevator;

import org.frc5687.robot.subsystems.SubsystemIO;

public interface ElevatorIO extends SubsystemIO<ElevatorInputs, ElevatorOutputs> {
    void setPID(double kP, double kI, double kD, double kV, double kS, double kA, double kG);

    default void resetSafety() {}
}
