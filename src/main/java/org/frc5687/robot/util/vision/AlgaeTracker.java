package org.frc5687.robot.util.vision;

import java.lang.constant.Constable;
import java.util.ArrayList;
import java.util.List;

import org.frc5687.robot.util.TunableDouble;
import org.frc5687.robot.Constants;

public class AlgaeTracker {
    private static AlgaeTracker _instance;
    private static List<Algae> _algae;

    private static final TunableDouble decay = new TunableDouble("AlgaeTracker", "decay", 0.95);
    private static final TunableDouble closeEnough = new TunableDouble("AlgaeTracker", "closeEnough", 0.3);
    private static final TunableDouble posLerp = new TunableDouble("AlgaeTracker", "posLerp", 0.5);
    private static final TunableDouble velLerp = new TunableDouble("AlgaeTracker", "velLerp", 0.5);

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
        for (var obs : observations) {
            if (obs.getClassId() == ALGAE_CLASS_ID)
                processObservation(obs);
        }
        // iterate backwards, updating all algae & deleting improbable algae
        for (int i = _algae.size() - 1; i >= 0; i--) {
            var algae = _algae.get(i);
            algae.x += algae.xVel * Constants.UPDATE_PERIOD;
            algae.y += algae.yVel * Constants.UPDATE_PERIOD;
            algae.prob *= decay.get();
            if (algae.prob < 0.2) {
                _algae.remove(i); // remove the index i
            }
        }
    }

    private synchronized void processObservation(NeuralPipelineObservation obs) {
        double closestDistance = Double.POSITIVE_INFINITY;
        int closestIndex = -1;
        for (int i = 0; i < _algae.size(); i++) {
            var alg = _algae.get(i);
            double dist = Math.hypot(alg.x - obs.getX(), alg.y - obs.getY());
            if (dist < closestDistance) {
                closestDistance = dist;
                closestIndex = i;
            }
        }

        if (closestDistance < closeEnough.get()) {
            var algae = _algae.get(closestIndex);
            double errorX = obs.getX() - algae.x;
            double errorY = obs.getY() - algae.y;
            algae.x += posLerp.get() * errorX;
            algae.y += posLerp.get() * errorY;
            algae.xVel += velLerp.get() / Constants.UPDATE_PERIOD * errorX;
            algae.yVel += velLerp.get() / Constants.UPDATE_PERIOD * errorY;
            algae.prob = algae.prob * 0.7 + 1.0 * 0.3;
        } else {
            _algae.add(new Algae(obs.getX(), obs.getY()));
        }
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
    };
}
