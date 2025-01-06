package org.frc5687.robot.subsystems;

import org.frc5687.robot.util.BaseInputs;
import org.frc5687.robot.util.BaseOutputs;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface SubsystemIO<Inputs extends BaseInputs, Outputs extends BaseOutputs> {
    void updateInputs(Inputs inputs);
    void writeOutputs(Outputs Outputs);
    /* This is only really for zeroing sensors */
    default void reset() {}
}
