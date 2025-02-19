package org.frc5687.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import java.util.Optional;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.RobotStateManager.Geometry;
import org.frc5687.robot.RobotStateManager.RobotCoordinate;
import org.frc5687.robot.subsystems.OutliersSubsystem;
import org.frc5687.robot.util.TunableDouble;

public class ElevatorSubsystem extends OutliersSubsystem<ElevatorInputs, ElevatorOutputs> {
    private final RobotStateManager _robotState = RobotStateManager.getInstance();
    private final RobotContainer _container;

    private TunableDouble elevatorP = new TunableDouble("Elevator", "kP", Constants.Elevator.kP);
    private TunableDouble elevatorI = new TunableDouble("Elevator", "kI", Constants.Elevator.kI);
    private TunableDouble elevatorD = new TunableDouble("Elevator", "kD", Constants.Elevator.kD);
    private TunableDouble elevatorA = new TunableDouble("Elevator", "kA", Constants.Elevator.kA);
    private TunableDouble elevatorS = new TunableDouble("Elevator", "kS", Constants.Elevator.kS);
    private TunableDouble elevatorG = new TunableDouble("Elevator", "kG", Constants.Elevator.kG);
    private TunableDouble elevatorV = new TunableDouble("Elevator", "kV", Constants.Elevator.kV);

    private Optional<Double> _newDesiredPlatformHeight;

    public ElevatorSubsystem(RobotContainer container, ElevatorIO io) {
        super(container, io, new ElevatorInputs(), new ElevatorOutputs());
        _container = container;
        _newDesiredPlatformHeight = Optional.empty();
    }

    @Override
    protected void processInputs() {
        _robotState.updatePlatform(_inputs.heightPositionMeters);
        _inputs.platformHeightMeters = _robotState.getPose(RobotCoordinate.ELEVATOR_TOP).getZ();
        _inputs.stagePose = _robotState.getPose(RobotCoordinate.ELEVATOR_STAGE);
        _inputs.platformPose = _robotState.getPose(RobotCoordinate.ELEVATOR_TOP);
    }

    @Override
    protected void periodic(ElevatorInputs inputs, ElevatorOutputs outputs) {
        // Update PID if needed
        if (elevatorP.hasChanged()
                || elevatorI.hasChanged()
                || elevatorD.hasChanged()
                || elevatorV.hasChanged()
                || elevatorA.hasChanged()
                || elevatorS.hasChanged()
                || elevatorG.hasChanged()) {
            _io.setPID(
                    elevatorP.get(),
                    elevatorI.get(),
                    elevatorD.get(),
                    elevatorV.get(),
                    elevatorS.get(),
                    elevatorA.get(),
                    elevatorG.get());
        }

        if (_newDesiredPlatformHeight.isPresent()) {
            double heightMeters = _newDesiredPlatformHeight.get();
            _outputs.desiredPlatformHeightWorldMeters = heightMeters;
            _outputs.desiredHeight = (heightMeters - Geometry.ELEVATOR_STAGE_TWO_HEIGHT) / 2.0;
            
            _newDesiredPlatformHeight = Optional.empty();
        }


    }

    public void setDesiredPlatformHeightWorld(double heightMeters) {
        heightMeters =
                MathUtil.clamp(
                        heightMeters,
                        Constants.Elevator.MIN_PLATFORM_HEIGHT,
                        Constants.Elevator.MAX_PLATFORM_HEIGHT);
        _newDesiredPlatformHeight = Optional.of(heightMeters);
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
}
