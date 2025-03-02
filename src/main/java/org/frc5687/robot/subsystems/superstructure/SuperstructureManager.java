package org.frc5687.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import java.util.function.Supplier;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.commands.algae.EjectAlgae;
import org.frc5687.robot.util.EpilogueLog;
import org.frc5687.robot.util.FieldConstants;

public class SuperstructureManager extends SubsystemBase implements EpilogueLog {
    public enum SuperstructureMode {
        CORAL,
        ALGAE
    }

    private final RobotContainer _container;
    private final RequestHandler _requestHandler;
    private boolean _forceQueueExecution = false;

    private SuperstructureMode _currentMode = SuperstructureMode.CORAL;

    public SuperstructureManager(RobotContainer container) {
        _container = container;
        _requestHandler = new RequestHandler(container);
    }

    public SuperstructureMode getCurrentMode() {
        return _currentMode;
    }

    public void setMode(SuperstructureMode mode) {
        if (_currentMode != mode) {
            System.out.println("Switching to " + mode + " mode");
            _currentMode = mode;
        }
    }

    public void toggleMode() {
        setMode(
                _currentMode == SuperstructureMode.CORAL
                        ? SuperstructureMode.ALGAE
                        : SuperstructureMode.CORAL);
    }

    public boolean isCoralMode() {
        return _currentMode == SuperstructureMode.CORAL;
    }

    public boolean isAlgaeMode() {
        return _currentMode == SuperstructureMode.ALGAE;
    }

    /**
     * This creates a request to the superstructure for a desired state while having the option to
     * queue.
     *
     * @param stateSupplier A supplier for the desired superstructure state positions
     * @param description debugging description of the action
     * @param type RequestType to queue or not to queue
     * @return Command for execution
     */
    public Command createRequest(
            Supplier<SuperstructureState> stateSupplier, String description, RequestType type) {
        return new FunctionalCommand(
                // init
                () ->
                        _requestHandler.handleNewRequest(
                                new SuperstructureRequest(
                                        stateSupplier.get(),
                                        type,
                                        () ->
                                                type != RequestType.QUEUED
                                                        || _forceQueueExecution
                                                        || isPositionNearReef()
                                                        || isElevatorGoingDown(stateSupplier.get()),
                                        description)),
                // execute
                () -> {},
                // end
                (interrupted) -> {
                    if (interrupted) {
                        System.out.println("Request interrupted: " + description);
                    }
                },
                // IsFinished
                () -> {
                    SuperstructureRequest activeRequest = _requestHandler.getActiveRequest();
                    return activeRequest == null;
                },
                this);
    }

    public Command createRequest(SuperstructureState placeState, RequestType type) {
        return createRequest(() -> placeState, "Set place height " + placeState.getElevator(), type);
    }

    public Command receiveFunnel(RequestType type) {
        // receiving from funnel, explicitly switch to CORAL mode
        return new SequentialCommandGroup(
                new InstantCommand(() -> setMode(SuperstructureMode.CORAL)),
                createRequest(Constants.SuperstructureGoals.RECEIVE_FROM_FUNNEL, type),
                new FunctionalCommand(
                        () -> {
                            _container.getCoral().setWheelMotorDutyCycle(0.3);
                        },
                        () -> {},
                        (interrupted) -> {
                            double currentPos = _container.getCoral().getWheelMotorPosition();
                            _container.getCoral().setWheelMotorPosition(currentPos + 1.5);
                        },
                        _container.getCoral()::isCoralDetected,
                        _container.getCoral()));
    }

    public Command receiveFunnelSim(RequestType type) {
        // receiving from funnel, explicitly switch to CORAL mode
        return new SequentialCommandGroup(
                new InstantCommand(() -> setMode(SuperstructureMode.CORAL)),
                createRequest(Constants.SuperstructureGoals.RECEIVE_FROM_FUNNEL, type));
    }

    public Command grabAlgae(SuperstructureState state, RequestType type) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> setMode(SuperstructureMode.ALGAE)),
                createRequest(state, type).until(() -> _container.getAlgae().isAlgaeDetected()));
    }

    public Command aimAtAlgaeNet() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> setMode(SuperstructureMode.ALGAE)),
                createRequest(Constants.SuperstructureGoals.BARGE_HELD, RequestType.IMMEDIATE),
                new EjectAlgae(_container.getAlgae()));
    }

    // Queue override controls
    public void forceQueueExecution() {
        _forceQueueExecution = true;
    }

    public void releaseQueueExecution() {
        _forceQueueExecution = false;
    }

    private boolean isPositionNearReef() {
        Translation2d currentPose = _container.getDrive().getPose().getTranslation();
        Translation2d reefCenter = FieldConstants.getAllianceSpecificReefCenter();
        return currentPose.getDistance(reefCenter) <= 2.0;
    }

    private boolean isElevatorGoingDown(SuperstructureState requestedState) {
        if (requestedState.getElevator().isEmpty()) return false; // FIXME is this the correct behavior?

        double elevatorHeight = _container.getElevator().getElevatorHeight();
        return requestedState.getElevator().get().getHeight() < elevatorHeight;
    }

    @Override
    public void periodic() {
        _requestHandler.execute();

        log("Mode", getCurrentMode());
        // TODO: I'm worried if a sensor gets hit and triggers always something bad will occur, maybe
        // always defualt to coral over algae if this occurs
        if (_container.getAlgae().isAlgaeDetected() && _currentMode != SuperstructureMode.ALGAE) {
            System.out.println("Auto-detecting ALGAE mode based on sensor");
            setMode(SuperstructureMode.ALGAE);
        } else if (_container.getCoral().isCoralDetected()
                && _currentMode != SuperstructureMode.CORAL) {
            System.out.println("Auto-detecting CORAL mode based on sensor");
            setMode(SuperstructureMode.CORAL);
        }
    }

    @Override
    public String getLogBase() {
        return "SuperstructureManager";
    }
}
