package org.frc5687.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import org.frc5687.robot.util.BaseInputs;
import org.frc5687.robot.util.BaseOutputs;

@Logged
public interface SubsystemIO<Inputs extends BaseInputs, Outputs extends BaseOutputs> {
    void updateInputs(Inputs inputs);

    void writeOutputs(Outputs outputs);

    /* This is only really for zeroing sensors */
    default void reset() {}

    default void setPID(
            double kP, double kI, double kD, double kV, double kS, double kA, double kG) {}
}
