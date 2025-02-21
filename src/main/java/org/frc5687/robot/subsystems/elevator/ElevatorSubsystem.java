package org.frc5687.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.RobotStateManager.RobotCoordinate;
import org.frc5687.robot.subsystems.OutliersSubsystem;
import org.frc5687.robot.util.TunableDouble;

public class ElevatorSubsystem extends OutliersSubsystem<ElevatorInputs, ElevatorOutputs> {
    private final RobotStateManager _robotState = RobotStateManager.getInstance();
    private final RobotContainer _container;
    private double _queuedHeight;
    private final ElevatorIO _elevatorIO;

    private TunableDouble elevatorP = new TunableDouble("Elevator", "kP", Constants.Elevator.kP);
    private TunableDouble elevatorI = new TunableDouble("Elevator", "kI", Constants.Elevator.kI);
    private TunableDouble elevatorD = new TunableDouble("Elevator", "kD", Constants.Elevator.kD);
    private TunableDouble elevatorA = new TunableDouble("Elevator", "kA", Constants.Elevator.kA);
    private TunableDouble elevatorS = new TunableDouble("Elevator", "kS", Constants.Elevator.kS);
    private TunableDouble elevatorG = new TunableDouble("Elevator", "kG", Constants.Elevator.kG);
    private TunableDouble elevatorV = new TunableDouble("Elevator", "kV", Constants.Elevator.kV);

    private Optional<Double> _newDesiredHeight;
    private boolean _previouslySafetyTripped = false;

    public ElevatorSubsystem(RobotContainer container, ElevatorIO io) {
        super(container, io, new ElevatorInputs(), new ElevatorOutputs());
        _elevatorIO = io;
        _container = container;
        _newDesiredHeight = Optional.empty();
        _queuedHeight = 0.0;
    }

    @Override
    protected void processInputs() {
        _robotState.updatePlatform(_inputs.heightPositionMeters);
        _inputs.stagePose = _robotState.getPose(RobotCoordinate.ELEVATOR_STAGE);
        _inputs.platformPose = _robotState.getPose(RobotCoordinate.ELEVATOR_TOP);

        // Log safety trips
        if (_inputs.isDisabled && !_previouslySafetyTripped) {
            DriverStation.reportError(
                    "ELEVATOR SAFETY TRIPPED: Position difference of "
                            + _inputs.eastWestPositionDifference
                            + "m exceeds threshold of "
                            + _inputs.positionDifferenceSafetyThreshold
                            + "m",
                    false);
            _previouslySafetyTripped = true;
        } else if (!_inputs.isDisabled && _previouslySafetyTripped) {
            _previouslySafetyTripped = false;
        }
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

        if (_newDesiredHeight.isPresent()) {
            _outputs.desiredHeight = _newDesiredHeight.get();
            _newDesiredHeight = Optional.empty();
        }
    }

    public void setDesiredHeight(ElevatorState state) {
        setDesiredHeight(state.getHeight());
    }

    public void setDesiredHeight(double heightMeters) {
        _outputs.controlMode = ElevatorControlMode.POSITION;
        heightMeters = MathUtil.clamp(heightMeters, Constants.Elevator.MIN_HEIGHT, Constants.Elevator.MAX_HEIGHT);
        _newDesiredHeight = Optional.of(heightMeters);
    }

    public boolean isAtDesiredPosition() {
        return Math.abs(_outputs.desiredHeight - _inputs.heightPositionMeters) < 0.01;
    }

    public double getElevatorHeight() {
        return _inputs.heightPositionMeters;
    }

    public void setVoltage(double voltage) {
        _outputs.controlMode = ElevatorControlMode.VOLTAGE;
        _outputs.voltageCommandEast = voltage;
        _outputs.voltageCommandWest = voltage;
    }

    public double getAverageCurrent() {
        return (_inputs.platformMotorCurrents[0] + _inputs.platformMotorCurrents[1]) / 2.0;
    }

    public double getLaserDistance() {
        return _inputs.laserSensorElevatorHeightMeters;
    }

    public boolean isAtState(ElevatorState state) {
        double heightDiff = Math.abs(state.getHeight() - getElevatorHeight());
        // 1 cm tolerance
        boolean isWithinPositionTolerance = heightDiff < 0.01;
        // Also check velocity is close to zero (added)
        boolean isVelocityNearZero = Math.abs(_inputs.firstStageVelocityMPS) < 0.01;

        return isWithinPositionTolerance && isVelocityNearZero;
    }

    /**
     * @return Whether the elevator is currently disabled due to a safety trip
     */
    public boolean isSafetyTripped() {
        return _inputs.isDisabled;
    }

    public void resetSafety() {
        if (_inputs.isDisabled) {
            _elevatorIO.resetSafety();
        }
    }

    public double getPositionDifference() {
        return _inputs.eastWestPositionDifference;
    }

    public double getPositionDifferenceThreshold() {
        return _inputs.positionDifferenceSafetyThreshold;
    }

    public String getSafetyStatus() {
        return _inputs.safetyStatus;
    }
}
