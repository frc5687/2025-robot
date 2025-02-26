package org.frc5687.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import java.util.function.Supplier;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotStateManager;
import org.photonvision.EstimatedRobotPose;

public class QuestNavPoseEstimator implements EpilogueLog {
    private Transform2d _originToFieldEstimate;
    private final QuestNav _questNav;
    private final Supplier<ChassisSpeeds> _chassisSpeedsSupplier;
    private static final Pose2d FIELD_TO_ORIGIN =
            new Pose2d(2.5, 1.75, new Rotation2d(Units.degreesToRadians(32)));

    public QuestNavPoseEstimator(QuestNav questNav, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        _originToFieldEstimate = null;
        _questNav = questNav;
        _chassisSpeedsSupplier = chassisSpeedsSupplier;
    }

    public void addVisionMeasurement(EstimatedRobotPose visionPose, double timestamp) {
        Optional<Pose2d> questPose = getRawQuestPose(timestamp - 0.02);
        if (questPose.isEmpty()) return;
        Transform2d originToQuest = new Transform2d(new Pose2d(), questPose.get());
        Transform2d visionFieldToRobot =
                new Transform2d(new Pose2d(), visionPose.estimatedPose.toPose2d());
        Transform2d fieldToQuest = visionFieldToRobot.plus(Constants.Vision.ROBOT_TO_QUEST);
        Transform2d originToField = originToQuest.plus(fieldToQuest.inverse());
        log("Vision fieldToRobot", visionFieldToRobot, Transform2d.struct);
        // TODO get real x y theta error for each measurement and use sheets
        log(
                "Interpolated fieldToRobot",
                RobotStateManager.getInstance().getQuestPose(timestamp - 0.02).get(),
                Pose2d.struct);
        log("Measured originToField", originToField, Transform2d.struct);

        if (_originToFieldEstimate == null) {
            _originToFieldEstimate = originToField;
            return;
        }

        ChassisSpeeds speeds = _chassisSpeedsSupplier.get();
        double angularVelocity = Math.abs(speeds.omegaRadiansPerSecond);
        double linearVelocity = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        Matrix<N3, N1> stddevs = VisionSTDFilter.calculateVisionStdDevs(visionPose);
        double speedStd = 0.02 * linearVelocity + 0.05 * angularVelocity;
        log("raw stddev", stddevs, Matrix.getStruct(Nat.N3(), Nat.N1()));
        log("speed stddev", speedStd);
        double xyStd = stddevs.get(0, 0) + speedStd;
        double thetaStd = stddevs.get(2, 0) / 2.0 + speedStd;

        double newX =
                addMeasurementToAverage(originToField.getX(), _originToFieldEstimate.getX(), xyStd);
        double newY =
                addMeasurementToAverage(originToField.getY(), _originToFieldEstimate.getY(), xyStd);

        double averageRotationVectorX = _originToFieldEstimate.getRotation().getCos();
        double averageRotationVectorY = _originToFieldEstimate.getRotation().getSin();
        double guessRotationVectorX = originToField.getRotation().getCos();
        double guessRotationVectorY = originToField.getRotation().getSin();

        double newRotationVectorX =
                addMeasurementToAverage(guessRotationVectorX, averageRotationVectorX, thetaStd);
        double newRotationVectorY =
                addMeasurementToAverage(guessRotationVectorY, averageRotationVectorY, thetaStd);
        Rotation2d newRotation = new Rotation2d(newRotationVectorX, newRotationVectorY);

        _originToFieldEstimate = new Transform2d(newX, newY, newRotation);
        log("Estimated quest origin to field origin", _originToFieldEstimate, Transform2d.struct);
    }

    private double addMeasurementToAverage(double newMeasurement, double prevAverage, double stddev) {
        double averagingFactor = 0.04 * Math.exp(-10 * stddev); // arbitrary function i made up
        return newMeasurement * averagingFactor + prevAverage * (1 - averagingFactor);
    }

    private Optional<Pose2d> getRawQuestPose(double timestamp) {
        var pose = RobotStateManager.getInstance().getQuestPose(timestamp);
        if (pose.isEmpty()) return Optional.empty();
        return Optional.of(
                FIELD_TO_ORIGIN.plus(
                        new Transform2d(new Pose2d(), pose.get()).plus(Constants.Vision.ROBOT_TO_QUEST)));
    }

    public Pose2d getEstimatedRobotPose() {
        if (_originToFieldEstimate == null) return new Pose2d(); // scuffed
        Transform2d estimatedQuestPose =
                _originToFieldEstimate
                        .inverse()
                        .plus(new Transform2d(new Pose2d(), getRawQuestPose(Timer.getFPGATimestamp()).get()));
        return new Pose2d().plus(estimatedQuestPose).plus(Constants.Vision.ROBOT_TO_QUEST.inverse());
    }

    @Override
    public String getLogBase() {
        return "QuestNavPoseEstimator";
    }
}
