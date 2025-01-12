package org.frc5687.robot.subsystems.drive;

import org.frc5687.robot.subsystems.SubsystemIO;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface DriveIO extends SubsystemIO<DriveInputs, DriveOutputs> {
    public void runCharacterization(double output);
}
