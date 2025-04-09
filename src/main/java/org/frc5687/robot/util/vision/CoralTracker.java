package org.frc5687.robot.util.vision;

import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.RobotStateManager.RobotCoordinate;
import org.frc5687.robot.util.EpilogueLog;
import org.frc5687.robot.util.TunableDouble;

public class CoralTracker implements EpilogueLog {
    private static CoralTracker _instance;
    private static List<Coral> _coral;

    private static final TunableDouble decay = new TunableDouble("CoralTracker", "decay", 0.95);
    private static final TunableDouble closeEnough =
            new TunableDouble("CoralTracker", "closeEnough", 0.3);
    private static final TunableDouble posLerp = new TunableDouble("CoralTracker", "posLerp", 0.5);
    private static final TunableDouble velLerp = new TunableDouble("CoralTracker", "velLerp", 0.005);

    private static final TunableDouble weighting = new TunableDouble("CoralTracking", "weighting", 4);

    private static final int CORAL_CLASS_ID = 1;

    private CoralTracker() {
        _coral = new ArrayList<>();
    }

    public static synchronized CoralTracker getInstance() {
        if (_instance == null) {
            _instance = new CoralTracker();
        }
        return _instance;
    }

    public synchronized void update(List<NeuralPipelineObservation> observations) {
        if (observations == null) {
            return;
        }
        for (var obs : observations) {
            if (obs.getClassId() == CORAL_CLASS_ID) processObservation(obs);
        }
        List<Pose2d> poses = new ArrayList<>();
        // iterate backwards, updating all coral & deleting improbable coral
        for (int i = _coral.size() - 1; i >= 0; i--) {
            var coral = _coral.get(i);
            coral.x += coral.xVel * Constants.UPDATE_PERIOD;
            coral.y += coral.yVel * Constants.UPDATE_PERIOD;
            coral.prob *= decay.get();
            if (coral.prob < 0.2) {
                _coral.remove(i); // remove the index i
            } else {
                poses.add(new Pose2d(coral.x, coral.y, new Rotation2d()));
            }
        }
        log("coral poses", poses, Pose2d.struct, Importance.CRITICAL);
    }

    private synchronized void processObservation(NeuralPipelineObservation obs) {
        Transform2d robotToDetection = new Transform2d(obs.getX(), obs.getY(), Rotation2d.kZero);
        Pose2d worldToRobot =
                RobotStateManager.getInstance().getPose(RobotCoordinate.ROBOT_BASE_SWERVE).toPose2d();
        Pose2d worldToDetection = worldToRobot.plus(robotToDetection);
        double x = worldToDetection.getX();
        double y = worldToDetection.getY();

        double closestDistance = Double.POSITIVE_INFINITY;
        int closestIndex = -1;
        for (int i = 0; i < _coral.size(); i++) {
            var alg = _coral.get(i);
            double dist = Math.hypot(alg.x - x, alg.y - y);
            if (dist < closestDistance) {
                closestDistance = dist;
                closestIndex = i;
            }
        }

        if (closestDistance < closeEnough.get()) {
            var coral = _coral.get(closestIndex);
            double errorX = x - coral.x;
            double errorY = y - coral.y;
            coral.x += posLerp.get() * errorX;
            coral.y += posLerp.get() * errorY;
            coral.xVel += velLerp.get() / Constants.UPDATE_PERIOD * errorX;
            coral.yVel += velLerp.get() / Constants.UPDATE_PERIOD * errorY;
            coral.prob = coral.prob * 0.7 + 1.0 * 0.3;
        } else {
            _coral.add(new Coral(x, y));
        }
    }

    public Optional<Translation2d> getClosestCoral(Pose2d robot) {
        var minDist = Double.POSITIVE_INFINITY;
        Optional<Translation2d> closestCoral = Optional.empty();
        for (var coral : _coral) {
            var dist = Math.hypot(coral.x - robot.getX(), coral.y - robot.getY());
            var angleToCoral = new Rotation2d(coral.x - robot.getX(), coral.y - robot.getY());
            var angleErr =
                    Math.pow(robot.getRotation().plus(Rotation2d.k180deg).minus(angleToCoral).getRadians(), 2)
                            * weighting.get();
            if (dist + angleErr < minDist) {
                minDist = dist + angleErr;
                closestCoral = Optional.of(new Translation2d(coral.x, coral.y));
            }
        }
        if (closestCoral.isPresent())
            log(
                    "closest coral",
                    new Pose2d(closestCoral.get(), Rotation2d.kZero),
                    Pose2d.struct,
                    Importance.CRITICAL);
        return closestCoral;
    }

    class Coral {
        public double x;
        public double y;
        public double xVel;
        public double yVel;
        public double prob;

        Coral(double x, double y) {
            this.x = x;
            this.y = y;
            this.xVel = 0;
            this.yVel = 0;
            this.prob = 0.7;
        }
    }

    @Override
    public String getLogBase() {
        return "CoralTracker";
    }
}
