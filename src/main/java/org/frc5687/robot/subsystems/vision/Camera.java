package org.frc5687.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;
import org.frc5687.robot.util.vision.AprilTagObservation;
import org.frc5687.robot.util.vision.NeuralPipelineObservation;

public abstract class Camera {
    /**
     * @return the results of the last processed frame
     */
    public abstract VisionResult getLatestResult();

    /**
     * @param index pipeline index (-1 for driver mode in PhotonVision)
     */
    public abstract void setPipelineIndex(int index);

    public abstract int getCurrentPipelineIndex();

    public record VisionResult(
            List<AprilTagObservation> tags,
            List<NeuralPipelineObservation> neuralObservations,
            Optional<RobotPoseEstimate> poseEstimate) {}
}
