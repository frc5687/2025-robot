package org.frc5687.robot.subsystems.drive.modules;

import org.frc5687.robot.subsystems.SubsystemIO;

public interface SwerveModuleIO extends SubsystemIO<SwerveModuleInputs, SwerveModuleOutputs> {
    void reset(); // Needed for resetting absolute encoder offset
}

