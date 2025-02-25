package org.frc5687.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import java.util.function.Supplier;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.util.FieldConstants;

public class SuperstructureManager extends SubsystemBase {
    private final RobotContainer _container;
    private final RequestHandler _requestHandler;
    private boolean _forceQueueExecution = false;

    public SuperstructureManager(RobotContainer container) {
        _container = container;
        _requestHandler = new RequestHandler(container);
    }

    /**
     * This crates a request to the superstructure for a desired state and motor values while having
     * the option to queue
     *
     * @param stateSupplier A supplier for the desired superstructure state positions (elevator
     *     height, arm angles, etc)
     * @param motors The motor values for certian actions (coral eject, algae hold etc)
     * @param description debugging description of the action
     * @param type RequestType to queue or not to queue (that is the question is it not)?
     * @return
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
        return new SequentialCommandGroup(
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
        return createRequest(Constants.SuperstructureGoals.RECEIVE_FROM_FUNNEL, type);
    }

    public Command grabAlgae(SuperstructureState state, RequestType type) {
        return createRequest(state, type).until(() -> _container.getAlgae().isAlgaeDetected());
    }

    public Command groundIntakeAlgae(SuperstructureState state, RequestType type) {
        return createRequest(state, type).until(() -> _container.getAlgae().isAlgaeDetected());
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
        return currentPose.getDistance(reefCenter) <= 3.5;
    }

    private boolean isElevatorGoingDown(SuperstructureState requestedState) {
        if (requestedState.getElevator().isEmpty()) return false; // FIXME is this the correct behavior?

        double elevatorHeight = _container.getElevator().getElevatorHeight();
        return requestedState.getElevator().get().getHeight() < elevatorHeight;
    }

    @Override
    public void periodic() {
        _requestHandler.execute();
    }

    // private SuperstructureState getCurrentGoal() {
    //     return new SuperstructureState(
    //             _container.getElevator().getDesiredState(),
    //             _container.getCoral().getDesiredState(),
    //             _container.getAlgae().getDesiredState(),
    //             IntakeState.IDLE); // TODO FIX INTAKE
    // }
}
