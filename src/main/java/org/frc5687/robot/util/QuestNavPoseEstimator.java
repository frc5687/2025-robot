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
import java.util.Optional;
import java.util.function.Supplier;
import org.frc5687.robot.Constants;
import org.photonvision.EstimatedRobotPose;

public class QuestNavPoseEstimator implements EpilogueLog {
    private Transform2d _originToFieldEstimate;
    private final QuestNav _questNav;
    private final Supplier<ChassisSpeeds> _chassisSpeedsSupplier;
    private static final Pose2d FIELD_TO_ORIGIN =
            new Pose2d(2.5, 1.75, new Rotation2d(Units.degreesToRadians(32)));

    private double xyFactorTotal;
    private double thetaFactorTotal;
    private int measurements;

    public QuestNavPoseEstimator(QuestNav questNav, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        _originToFieldEstimate = new Transform2d();
        _questNav = questNav;
        _chassisSpeedsSupplier = chassisSpeedsSupplier;
        xyFactorTotal = 100.0;
        thetaFactorTotal = 100.0;
        measurements = 0;
    }

    public void addVisionMeasurement(EstimatedRobotPose visionPose, double timestamp) {
        if (_questNav.timeSinceLastUpdate() > 0.040) return;
        Optional<Pose2d> questPose = _questNav.getQuestPose(timestamp);
        if (questPose.isEmpty()) return;
        log("Interpolated fieldToRobot", questPose.get(), Pose2d.struct);
        Transform2d originToQuest = new Transform2d(new Pose2d(), questPose.get());
        Transform2d visionFieldToRobot =
                new Transform2d(new Pose2d(), visionPose.estimatedPose.toPose2d());
        Transform2d fieldToQuest = visionFieldToRobot.plus(Constants.Vision.ROBOT_TO_QUEST);
        Transform2d originToField = originToQuest.plus(fieldToQuest.inverse());
        Transform2d estimatedError = _originToFieldEstimate.plus(originToField.inverse());
        // if (measurements > 10 && estimatedError.getTranslation().getNorm() > 0.3) {
        //     System.out.println("error too high " + estimatedError.getTranslation().getNorm());
        //     return;
        // }

        log("Vision fieldToRobot", visionFieldToRobot, Transform2d.struct);
        // TODO get real x y theta error for each measurement and use sheets
        log("Measured originToField", originToField, Transform2d.struct);

        ChassisSpeeds speeds = _chassisSpeedsSupplier.get();
        double angularVelocity = Math.abs(speeds.omegaRadiansPerSecond);
        double linearVelocity = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        Matrix<N3, N1> stddevs = VisionSTDFilter.calculateVisionStdDevs(visionPose);
        double speedStd = 0.02 * linearVelocity + 0.05 * angularVelocity;
        log("raw stddev", stddevs, Matrix.getStruct(Nat.N3(), Nat.N1()));
        log("speed stddev", speedStd);
        double xyStd = stddevs.get(0, 0) + speedStd;
        double thetaStd = stddevs.get(2, 0) / 2.0 + speedStd;
        double distance = questPose.get().getTranslation().getNorm();
        double xyFactor = Math.exp(-10 * xyStd); // arbitrary function i made up
        double thetaFactor = Math.exp(-10 * thetaStd) * (1 + distance); // arbitrary function i made up

        double newX =
                addMeasurementToAverage(
                        originToField.getX(), _originToFieldEstimate.getX(), xyFactor, xyFactorTotal);
        double newY =
                addMeasurementToAverage(
                        originToField.getY(), _originToFieldEstimate.getY(), xyFactor, xyFactorTotal);

        double averageRotationVectorX = _originToFieldEstimate.getRotation().getCos();
        double averageRotationVectorY = _originToFieldEstimate.getRotation().getSin();
        double guessRotationVectorX = originToField.getRotation().getCos();
        double guessRotationVectorY = originToField.getRotation().getSin();

        double newRotationVectorX =
                addMeasurementToAverage(
                        guessRotationVectorX, averageRotationVectorX, thetaFactor, thetaFactorTotal);
        double newRotationVectorY =
                addMeasurementToAverage(
                        guessRotationVectorY, averageRotationVectorY, thetaFactor, thetaFactorTotal);
        Rotation2d newRotation = new Rotation2d(newRotationVectorX, newRotationVectorY);

        // xyFactorTotal += xyFactor;
        // thetaFactorTotal += thetaFactor;
        measurements += 1;

        _originToFieldEstimate = new Transform2d(newX, newY, newRotation);
        log("Estimated quest origin to field origin", _originToFieldEstimate, Transform2d.struct);
        log("Quest Start Pose", _originToFieldEstimate.inverse(), Transform2d.struct);
        log(
                "Robot Start Pose",
                _originToFieldEstimate.inverse().plus(Constants.Vision.ROBOT_TO_QUEST.inverse()),
                Transform2d.struct);
    }

    private double addMeasurementToAverage(
            double newMeasurement, double prevAverage, double newFactor, double prevFactorTotal) {
        double sum = prevAverage * prevFactorTotal + newMeasurement * newFactor;
        return sum / (prevFactorTotal + newFactor);
    }

    //     private void resetRobotPose(Pose2d robotPose) {
    //         if (_questNav.timeSinceLastUpdate() > 0.040) {
    //                 System.err.println("Failed to reset pose. Quest is down.");
    //                 return;
    //         };
    //         Pose2d questPose = _questNav.getQuestPose();
    //         Transform2d originToQuest = new Transform2d(new Pose2d(), questPose);
    //         Transform2d fieldToRobot = new Transform2d(new Pose2d(), robotPose);
    //         Transform2d fieldToQuest = visionFieldToRobot.plus(Constants.Vision.ROBOT_TO_QUEST);
    //         Transform2d originToField = originToQuest.plus(fieldToQuest.inverse());
    //     }

    public Pose2d getEstimatedRobotPose() {
        if (_originToFieldEstimate == null) return new Pose2d(); // scuffed
        Transform2d estimatedQuestPose =
                _originToFieldEstimate
                        .inverse()
                        .plus(new Transform2d(new Pose2d(), _questNav.getQuestPose()));
        return new Pose2d().plus(estimatedQuestPose).plus(Constants.Vision.ROBOT_TO_QUEST.inverse());
    }

    @Override
    public String getLogBase() {
        return "QuestNavPoseEstimator";
    }
}
