package org.frc5687.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
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

    private final PIDController _pitchController;
    private final PIDController _rollController;

    private TunableDouble elevatorP = new TunableDouble("Elevator", "kP", Constants.Elevator.STAB_kP);
    private TunableDouble elevatorI = new TunableDouble("Elevator", "kI", Constants.Elevator.STAB_kI);
    private TunableDouble elevatorD = new TunableDouble("Elevator", "kD", Constants.Elevator.STAB_kD);
    private TunableDouble elevatorA = new TunableDouble("Elevator", "kA", Constants.Elevator.STAB_kA);
    private TunableDouble elevatorS = new TunableDouble("Elevator", "kS", Constants.Elevator.STAB_kS);
    private TunableDouble elevatorG = new TunableDouble("Elevator", "kG", Constants.Elevator.STAB_kG);
    private TunableDouble elevatorV = new TunableDouble("Elevator", "kv", Constants.Elevator.STAB_kA);

    private TunableDouble PITCH_kP =
            new TunableDouble("Elevator", "Pitch kP", Constants.Elevator.PITCH_kP);
    private TunableDouble PITCH_kD =
            new TunableDouble("Elevator", "Pitch kD", Constants.Elevator.PITCH_kD);
    private TunableDouble ROLL_kP =
            new TunableDouble("Elevator", "Roll kP", Constants.Elevator.ROLL_kP);
    private TunableDouble ROLL_kD =
            new TunableDouble("Elevator", "Roll kD", Constants.Elevator.ROLL_kD);

    private TunableDouble backlashOffset = new TunableDouble("ElevatorHardware", "backlash", 0.00);

    private Optional<Double> _newDesiredPlatformHeight;

    public ElevatorSubsystem(RobotContainer container, ElevatorIO io) {
        super(container, io, new ElevatorInputs(), new ElevatorOutputs());
        this.setToSeparateControl(true);
        _container = container;

        _pitchController = new PIDController(PITCH_kP.get(), 0.0, PITCH_kD.get());
        _rollController = new PIDController(ROLL_kP.get(), 0.0, ROLL_kD.get());
        _pitchController.setSetpoint(0.0);
        _rollController.setSetpoint(0.0);
        _pitchController.setTolerance(Units.degreesToRadians(0.1));
        _rollController.setTolerance(Units.degreesToRadians(0.1));
        _newDesiredPlatformHeight = Optional.empty();
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

        _pitchController.setP(PITCH_kP.get());
        _pitchController.setD(PITCH_kD.get());
        _pitchController.setP(ROLL_kP.get());
        _pitchController.setD(ROLL_kD.get());

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

        if (_newDesiredPlatformHeight.isPresent()) {
            double heightMeters = _newDesiredPlatformHeight.get();
            _outputs.desiredPlatformHeightWorldMeters = heightMeters;
            _outputs.desiredStageHeight = (heightMeters - Geometry.ELEVATOR_STAGE_TWO_HEIGHT) / 2.0;
            _newDesiredPlatformHeight = Optional.empty();
        }

        outputs.northEastStageHeight =
                outputs.desiredStageHeight
                        + (outputs.desiredStageHeight * backlashOffset.get())
                        - pitchCorrection
                        - rollCorrection;
        outputs.northWestStageHeight =
                outputs.desiredStageHeight
                        + (outputs.desiredStageHeight * backlashOffset.get())
                        - pitchCorrection
                        + rollCorrection;
        outputs.southWestStageHeight =
                outputs.desiredStageHeight
                        - (outputs.desiredStageHeight * backlashOffset.get())
                        + pitchCorrection
                        + rollCorrection;
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

    public void processWithSeparateControl() {
        if (!this.isSeparateControl()) {
            throw new IllegalStateException("Cannot call when separate control is disabled");
        }
        process();
    }
}
