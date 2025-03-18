package org.frc5687.robot.subsystems.drive;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import org.frc5687.robot.subsystems.SubsystemIO;

@Logged
public interface DriveIO extends SubsystemIO<DriveInputs, DriveOutputs> {
    void runCharacterization(double output);

    default void setYaw(Rotation2d heading) {}
    ;
}
