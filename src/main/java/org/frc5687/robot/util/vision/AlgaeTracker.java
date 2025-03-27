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

public class AlgaeTracker implements EpilogueLog {
    private static AlgaeTracker _instance;
    private static List<Algae> _algae;

    private static final TunableDouble decay = new TunableDouble("AlgaeTracker", "decay", 0.95);
    private static final TunableDouble closeEnough =
            new TunableDouble("AlgaeTracker", "closeEnough", 0.3);
    private static final TunableDouble posLerp = new TunableDouble("AlgaeTracker", "posLerp", 0.5);
    private static final TunableDouble velLerp = new TunableDouble("AlgaeTracker", "velLerp", 0.005);

    private static final int ALGAE_CLASS_ID = 0;

    private AlgaeTracker() {
        _algae = new ArrayList<>();
    }

    public static synchronized AlgaeTracker getInstance() {
        if (_instance == null) {
            _instance = new AlgaeTracker();
        }
        return _instance;
    }

    public synchronized void update(List<NeuralPipelineObservation> observations) {
        if (observations == null) {
            return;
        }
        for (var obs : observations) {
            if (obs.getClassId() == ALGAE_CLASS_ID) processObservation(obs);
        }
        List<Pose2d> poses = new ArrayList<>();
        // iterate backwards, updating all algae & deleting improbable algae
        for (int i = _algae.size() - 1; i >= 0; i--) {
            var algae = _algae.get(i);
            algae.x += algae.xVel * Constants.UPDATE_PERIOD;
            algae.y += algae.yVel * Constants.UPDATE_PERIOD;
            algae.prob *= decay.get();
            if (algae.prob < 0.2) {
                _algae.remove(i); // remove the index i
            } else {
                poses.add(new Pose2d(algae.x, algae.y, new Rotation2d()));
            }
        }
        log("algae poses", poses, Pose2d.struct, Importance.CRITICAL);
    }

    private synchronized void processObservation(NeuralPipelineObservation obs) {
        if (obs.getClassId() != 0) return; // algae only
        Transform2d robotToDetection = new Transform2d(obs.getX(), obs.getY(), Rotation2d.kZero);
        Pose2d worldToRobot =
                RobotStateManager.getInstance().getPose(RobotCoordinate.ROBOT_BASE_SWERVE).toPose2d();
        Pose2d worldToDetection = worldToRobot.plus(robotToDetection);
        double x = worldToDetection.getX();
        double y = worldToDetection.getY();

        double closestDistance = Double.POSITIVE_INFINITY;
        int closestIndex = -1;
        for (int i = 0; i < _algae.size(); i++) {
            var alg = _algae.get(i);
            double dist = Math.hypot(alg.x - x, alg.y - y);
            if (dist < closestDistance) {
                closestDistance = dist;
                closestIndex = i;
            }
        }

        if (closestDistance < closeEnough.get()) {
            var algae = _algae.get(closestIndex);
            double errorX = x - algae.x;
            double errorY = y - algae.y;
            algae.x += posLerp.get() * errorX;
            algae.y += posLerp.get() * errorY;
            algae.xVel += velLerp.get() / Constants.UPDATE_PERIOD * errorX;
            algae.yVel += velLerp.get() / Constants.UPDATE_PERIOD * errorY;
            algae.prob = algae.prob * 0.7 + 1.0 * 0.3;
        } else {
            _algae.add(new Algae(x, y));
        }
    }

    public Optional<Translation2d> getClosestAlgae(Translation2d robot) {
        var minDist = Double.POSITIVE_INFINITY;
        Optional<Translation2d> closestAlgae = Optional.empty();
        for (var algae : _algae) {
            var dist = Math.hypot(algae.x - robot.getX(), algae.y - robot.getY());
            if (dist < minDist) {
                minDist = dist;
                closestAlgae = Optional.of(new Translation2d(algae.x, algae.y));
            }
        }
        if (closestAlgae.isPresent())
            log(
                    "cloest algae",
                    new Pose2d(closestAlgae.get(), Rotation2d.kZero),
                    Pose2d.struct,
                    Importance.CRITICAL);
        return closestAlgae;
    }

    class Algae {
        public double x;
        public double y;
        public double xVel;
        public double yVel;
        public double prob;

        Algae(double x, double y) {
            this.x = x;
            this.y = y;
            this.xVel = 0;
            this.yVel = 0;
            this.prob = 0.7;
        }
    }

    @Override
    public String getLogBase() {
        return "AlgaeTracker";
    }
}
