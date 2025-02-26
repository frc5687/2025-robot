package org.frc5687.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

public class QuestNavOdometrySource extends OdometrySource {
    private QuestNav _nav;

    public QuestNavOdometrySource(QuestNav nav) {
        _nav = nav;
    }

    @Override
    public Pose2d getOdometryPose() {
        return _nav.getRobotPose();
    }

    @Override
    public void resetPose(Pose2d pose) {
        // _nav.resetPose(pose);
    }
}
