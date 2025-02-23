package org.frc5687.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
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

    private TunableDouble elevatorP = new TunableDouble("Elevator", "kP", Constants.Elevator.kP);
    private TunableDouble elevatorI = new TunableDouble("Elevator", "kI", Constants.Elevator.kI);
    private TunableDouble elevatorD = new TunableDouble("Elevator", "kD", Constants.Elevator.kD);
    private TunableDouble elevatorA = new TunableDouble("Elevator", "kA", Constants.Elevator.kA);
    private TunableDouble elevatorS = new TunableDouble("Elevator", "kS", Constants.Elevator.kS);
    private TunableDouble elevatorG = new TunableDouble("Elevator", "kG", Constants.Elevator.kG);
    private TunableDouble elevatorV = new TunableDouble("Elevator", "kV", Constants.Elevator.kV);

    private Optional<Double> _newDesiredMotorHeight;

    public ElevatorSubsystem(RobotContainer container, ElevatorIO io) {
        super(container, io, new ElevatorInputs(), new ElevatorOutputs());
        _container = container;
        _newDesiredMotorHeight = Optional.empty();
    }

    @Override
    protected void processInputs() {
        _robotState.updatePlatform(_inputs.motorHeightMeters);
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

        if (_newDesiredMotorHeight.isPresent()) {
            _outputs.desiredMotorHeightMeters = _newDesiredMotorHeight.get();
            _newDesiredMotorHeight = Optional.empty();
        }
    }

    private void setDesiredMotorHeight(double heightMeters) {
        _outputs.controlMode = ElevatorControlMode.POSITION;
        heightMeters =
                MathUtil.clamp(heightMeters, Constants.Elevator.MIN_HEIGHT, Constants.Elevator.MAX_HEIGHT);
        _newDesiredMotorHeight = Optional.of(heightMeters);
    }

    public void setDesiredState(ElevatorState state) {
        setDesiredMotorHeight(state.getHeight());
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

    public boolean isAtDesiredPosition() {
        return Math.abs(_outputs.desiredMotorHeightMeters - _inputs.motorHeightMeters) < 0.01;
    }

    public void mapToClosestState() {
        ElevatorState closestState = ElevatorState.STOWED;
        double minDist = Double.MAX_VALUE;
        for (ElevatorState state : ElevatorState.values()) {
            double heightDiff = Math.abs(_inputs.motorHeightMeters - state.getHeight());
            if (heightDiff < minDist) {
                closestState = state;
                minDist = heightDiff;
            }
        }
        _inputs.elevatorState = closestState;
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

    public double getSignedTimeToSetpoint(double setpointMotorMeters) {
        double heightMeters = _inputs.motorHeightMeters;
        double secondsPerMeter = 2.0 / Constants.Elevator.MAX_HEIGHT;
        double signedDistance = setpointMotorMeters - heightMeters;

        double estimatedTime = signedDistance * secondsPerMeter + 2.0;
        log("DriveToPoseSmooth/timeItWillTakeElevator", estimatedTime);
        return estimatedTime;
    }
}
