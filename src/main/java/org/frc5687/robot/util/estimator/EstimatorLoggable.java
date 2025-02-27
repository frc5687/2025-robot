package org.frc5687.robot.util.estimator;

import org.frc5687.robot.util.EpilogueLog;

public interface EstimatorLoggable extends EpilogueLog {
    @Override
    default String getLogBase() {
        return "Estimator/" + getClass().getSimpleName();
    }
}
