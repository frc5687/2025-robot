package org.frc5687.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

public abstract class OdometrySource {
    public abstract Pose2d getOdometryPose();

    public abstract void resetPose(Pose2d pose);
}
