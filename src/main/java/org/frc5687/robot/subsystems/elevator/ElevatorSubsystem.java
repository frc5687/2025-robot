package org.frc5687.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
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

    private final PIDController _pitchController;
    private final PIDController _rollController;

    private TunableDouble elevatorP = new TunableDouble("Elevator", "kP", Constants.Elevator.HOLD_kP);
    private TunableDouble elevatorI = new TunableDouble("Elevator", "kI", Constants.Elevator.HOLD_kI);
    private TunableDouble elevatorD = new TunableDouble("Elevator", "kD", Constants.Elevator.HOLD_kD);
    private TunableDouble elevatorA = new TunableDouble("Elevator", "kA", Constants.Elevator.HOLD_kA);
    private TunableDouble elevatorS = new TunableDouble("Elevator", "kS", Constants.Elevator.HOLD_kS);
    private TunableDouble elevatorG = new TunableDouble("Elevator", "kG", Constants.Elevator.HOLD_kG);
    private TunableDouble elevatorV = new TunableDouble("Elevator", "kv", Constants.Elevator.HOLD_kA);

    public ElevatorSubsystem(RobotContainer container, ElevatorIO io) {
        super(container, io, new ElevatorInputs(), new ElevatorOutputs());
        this.setToSeparateControl(true);
        _container = container;

        _pitchController =
                new PIDController(Constants.Elevator.PITCH_kP, 0.0, Constants.Elevator.PITCH_kD);
        _rollController =
                new PIDController(Constants.Elevator.ROLL_kP, 0.0, Constants.Elevator.ROLL_kD);
        _pitchController.setSetpoint(0.0);
        _rollController.setSetpoint(0.0);
        _pitchController.setTolerance(Units.degreesToRadians(0.1));
        _rollController.setTolerance(Units.degreesToRadians(0.1));
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
                _inputs.platformRollRadians);

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

        double pitchCorrection = _pitchController.calculate(inputs.platformPitchRadians);
        double rollCorrection = _rollController.calculate(inputs.platformRollRadians);

        pitchCorrection =
                MathUtil.clamp(
                        pitchCorrection,
                        -Constants.Elevator.MAX_POSITION_CORRECTION,
                        Constants.Elevator.MAX_POSITION_CORRECTION);
        rollCorrection =
                MathUtil.clamp(
                        rollCorrection,
                        -Constants.Elevator.MAX_POSITION_CORRECTION,
                        Constants.Elevator.MAX_POSITION_CORRECTION);
    }

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
