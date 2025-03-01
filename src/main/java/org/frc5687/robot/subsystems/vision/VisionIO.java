package org.frc5687.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import org.frc5687.robot.subsystems.SubsystemIO;

public interface VisionIO extends SubsystemIO<VisionInputs, VisionOutputs> {
    default void resetCameraIMU(Rotation2d rotation) {}
    ;
}
