package org.frc5687.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.RobotStateManager.Geometry;
import org.frc5687.robot.RobotStateManager.RobotCoordinate;
import org.frc5687.robot.subsystems.OutliersSubsystem;

public class ElevatorSubsystem extends OutliersSubsystem<ElevatorInputs, ElevatorOutputs> {
    private final RobotStateManager _robotState = RobotStateManager.getInstance();

    public ElevatorSubsystem(RobotContainer container, ElevatorIO io) {
        super(container, io, new ElevatorInputs(), new ElevatorOutputs());
        this.setToSeparateControl(true);
    }

    @Override
    protected void processInputs() {
        _inputs.firstStagePositionMeters =
                (_inputs.stageNorthWestPositionMeters
                                + _inputs.stageNorthEastPositionMeters
                                + _inputs.stageSouthWestPositionMeters)
                        / 3.0;

        _robotState.updatePlatform(
                _inputs.firstStagePositionMeters,
                _inputs.platformPitchRadians,
                _inputs.platformPitchRadians);

        _inputs.platformHeightMeters = _robotState.getPose(RobotCoordinate.ELEVATOR_TOP).getZ();

        _inputs.stagePose = _robotState.getPose(RobotCoordinate.ELEVATOR_STAGE);
        _inputs.platformPose = _robotState.getPose(RobotCoordinate.ELEVATOR_TOP);
    }

    @Override
    protected void periodic(ElevatorInputs inputs, ElevatorOutputs outputs) {}

    public void setDesiredPlatformHeightWorld(double heightMeters) {
        heightMeters =
                MathUtil.clamp(
                        heightMeters,
                        Constants.Elevator.MIN_PLATFORM_HEIGHT,
                        Constants.Elevator.MAX_PLATFORM_HEIGHT);
        _outputs.desiredPlatformHeightWorldMeters = heightMeters;
        _outputs.desiredStageHeight = (heightMeters - Geometry.ELEVATOR_STAGE_TWO_HEIGHT) / 2.0;
        _outputs.desiredPlatformPitchRadians = 0.0;
        _outputs.desiredPlatformRollRadians = 0.0;
    }

    public void setDesiredState(ElevatorState state) {
        setDesiredPlatformHeightWorld(state.getValue());
        _outputs.desiredState = state;
    }

    public ElevatorState getCurrentState() {
        return _inputs.elevatorState;
    }

    public void setCurrentState(ElevatorState state) {
        _inputs.elevatorState = state;
    }

    public ElevatorState getDesiredState() {
        return _outputs.desiredState;
    }

    public double getPlatformWorldHeight() {
        return _inputs.platformPose.getZ();
    }

    public boolean isAtDesiredPosition() {
        return Math.abs(_outputs.desiredPlatformHeightWorldMeters - getPlatformWorldHeight()) < 0.01;
    }

    public void mapToClosestState() {
        ElevatorState closestState = ElevatorState.STOWED;
        double minDist = Double.MAX_VALUE;

        for (ElevatorState state : ElevatorState.values()) {
            double heightDiff = Math.abs(getPlatformWorldHeight() - state.getValue());
            if (heightDiff < minDist) {
                closestState = state;
                minDist = heightDiff;
            }
        }
        _inputs.elevatorState = closestState;
    }

    public void processWithSeparateControl() {
        if (!this.isSeparateControl()) {
            throw new IllegalStateException("Cannot call when separate control is disabled");
        }
        process();
    }
}
