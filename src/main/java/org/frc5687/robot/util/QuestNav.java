package org.frc5687.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import org.frc5687.robot.Constants;

/**
 * Interface with the QuestNav on VR headset for pose estimation. See
 * https://www.chiefdelphi.com/t/questnav-the-best-robot-pose-tracking-system-in-frc/
 */
public class QuestNav implements EpilogueLog {
    // Configure Network Tables topics (questnav/...) to communicate with the Quest HMD
    NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
    NetworkTable nt4Table = nt4Instance.getTable("questnav");
    private IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
    private IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();

    // Subscribe to the Network Tables questnav data topics
    private double prevTimestamp = 0;
    private DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
    private FloatArraySubscriber questPosition =
            nt4Table.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});
    private FloatArraySubscriber questQuaternion =
            nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[] {0.0f, 0.0f, 0.0f, 0.0f});
    private FloatArraySubscriber questEulerAngles =
            nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});
    private DoubleSubscriber questBatteryPercent =
            nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0f);

    // Pose of the Quest when the pose was reset
    // private Pose2d resetPoseOculus = new Pose2d();

    // Pose of the robot when the pose was reset
    // private Pose2d resetPoseRobot = new Pose2d();

    private final TimeInterpolatableBuffer<Pose2d> _rawQuestPoseBuffer;

    /* Constructor */
    public QuestNav() {
        // Zero the absolute 3D position of the robot (similar to long-pressing the quest logo)
        // if (questMiso.get() != 99) {
        //     questMosi.set(1);
        // }
        _rawQuestPoseBuffer = TimeInterpolatableBuffer.createBuffer(10.0);
    }

    /**
     * Gets the pose of the robot on the field
     *
     * @return pose of the robot
     */
    public Pose2d getRobotPose() {
        return getQuestPose().transformBy(Constants.Vision.ROBOT_TO_QUEST.inverse());
    }

    /**
     * Gets the pose of the Quest on the field
     *
     * @return pose of the Quest
     */
    // public Pose2d getQuestPose() {
    //     var rawPose = getUncorrectedOculusPose();
    //     // var poseRelativeToReset = rawPose.minus(resetPoseOculus);

    //     // return resetPoseRobot.transformBy(poseRelativeToReset);
    //     return rawPose;
    // }

    /*
     * Gets the battery percent of the Quest.
     *
     * @return battery percent of the Quest
     */
    public double getBatteryPercent() {
        return questBatteryPercent.get();
    }

    /**
     * Returns if the Quest is connected
     *
     * @return true if the Quest is connected
     */
    public boolean isConnected() {
        return ((RobotController.getFPGATime() - questBatteryPercent.getLastChange()) / 1000) < 250;
    }

    /**
     * Gets the raw Rotation3d of the Quest
     *
     * @return Rotation3d of the Quest, not adjusted for the reset pose
     */
    public Rotation3d getQuaternion() {
        float[] qqFloats = questQuaternion.get();
        return new Rotation3d(new Quaternion(qqFloats[0], qqFloats[1], qqFloats[2], qqFloats[3]));
    }

    /**
     * Gets the Quests's timestamp
     *
     * @return the Quest timestamp
     */
    public double getTimestamp() {
        return questTimestamp.get();
    }

    /**
     * Set the robot's pose on the field. This is useful to seed the robot to a known position. This
     * is usually called at the start of the autonomous period.
     *
     * @param newPose new robot pose
     */
    // public void resetPose(Pose2d newPose) {
    //     resetPoseOculus =
    //             getUncorrectedOculusPose().transformBy(Constants.Vision.ROBOT_TO_QUEST.inverse());
    //     resetPoseRobot = newPose;
    // }

    /**
     * Clean up questnav subroutine messages after processing on the headset. Call this each iteration
     * to reset the command after it has been processed.
     */
    public void cleanUpQuestNavMessages() {
        if (questMiso.get() == 99) {
            questMosi.set(0);
        }
    }

    /**
     * Gets the raw pose of the oculus, relative to the position where it started
     *
     * @return pose of the oculus
     */
    public Pose2d getQuestPose() {
        var eulerAngles = questEulerAngles.get();
        var rotation = Rotation2d.fromDegrees(-Math.IEEEremainder(eulerAngles[1], 360d));

        var questnavPosition = questPosition.get();
        var translation = new Translation2d(questnavPosition[2], -questnavPosition[0]);
        var pose = new Pose2d(translation, rotation);
        _rawQuestPoseBuffer.addSample(Timer.getFPGATimestamp(), pose);
        return pose;
    }

    public Optional<Pose2d> getQuestPose(double timestamp) {
        return _rawQuestPoseBuffer.getSample(timestamp);
    }

    public Pose3d getUncorrectedOculusPose3d() {

        var questnavPosition = questPosition.get();
        var translation =
                new Translation3d(questnavPosition[2], -questnavPosition[0], questnavPosition[1]);

        return new Pose3d(translation, getQuaternion());
    }

    @Override
    public String getLogBase() {
        return "QuestNav (java edition)";
    }

    /**
     * @return time since last data from quest (seconds)
     */
    public double timeSinceLastUpdate() {
        double questUpdateTimestamp = questTimestamp.getLastChange() / 1e6;
        double timeSinceLastUpdate = Timer.getFPGATimestamp() - questUpdateTimestamp;
        log("Time Since Quest Update (s)", timeSinceLastUpdate);
        return timeSinceLastUpdate;
    }
}
