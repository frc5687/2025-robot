package org.frc5687.robot.subsystems.superstructure;

import java.util.ArrayDeque;
import java.util.Optional;
import java.util.Queue;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.subsystems.coralarm.CoralState;
import org.frc5687.robot.subsystems.intake.IntakeState;
import org.frc5687.robot.util.EpilogueLog;

public class RequestHandler implements EpilogueLog {
    private final RobotContainer _container;

    private Queue<SuperstructureRequest> _activeRequests;
    private SuperstructureRequest _queuedRequest = null;

    public RequestHandler(RobotContainer container) {
        _container = container;
        _activeRequests = new ArrayDeque<>();
        _queuedRequest = null;
    }

    public void handleNewRequest(SuperstructureRequest request) {
        System.out.println("New request received: " + request.description());

        switch (request.type()) {
            case IMMEDIATE:
                handleImmediateRequest(request);
                break;
            case QUEUED:
                handleQueuedRequest(request);
                break;
        }
    }

    private void handleImmediateRequest(SuperstructureRequest request) {
        _queuedRequest = null; // should override the thing
        _activeRequests.clear();

        if (shouldCoralSadDance(request)) {
            // first get out of the way
            var step1 =
                    new SuperstructureState(
                            Optional.empty(), Optional.of(CoralState.IDLE), Optional.empty(), Optional.empty());
            addRequest(
                    new SuperstructureRequest(
                            step1, RequestType.IMMEDIATE, null, "outstretch hand (step 1)"));
            var step2 =
                    new SuperstructureState(
                            request.targetPosition().getElevator(),
                            Optional.of(CoralState.IDLE),
                            request.targetPosition().getAlgae(),
                            request.targetPosition().getIntake());
            addRequest(
                    new SuperstructureRequest(
                            step2, RequestType.IMMEDIATE, null, "raise with hand outstretched (step 2)"));
        }

        addRequest(request);
    }

    private boolean shouldCoralSadDance(SuperstructureRequest request) {
        var elevatorRequest = request.targetPosition().getElevator();
        if (elevatorRequest.isEmpty()) return false;

        var requestedElevatorHeight = elevatorRequest.get().getHeight();
        var currentElevatorHeight = _container.getElevator().getInputs().platformHeightMeters;
        return Math.abs(currentElevatorHeight - requestedElevatorHeight) > 0.03;
    }

    private void addRequest(SuperstructureRequest request) {
        System.out.println("-----");
        System.out.println("👂adding request: " + request.description());
        System.out.println("-----");
        _activeRequests.add(request);
    }

    private void handleQueuedRequest(SuperstructureRequest request) {
        _queuedRequest = request;
        System.out.println("-----");
        System.out.println("💟Request queued: " + request.description());
        System.out.println("-----");
    }

    public void execute() {
        checkAndStartQueuedRequest();
        executeActiveRequest();
    }

    private void checkAndStartQueuedRequest() {
        if (_queuedRequest != null && _queuedRequest.driveCondition().get()) {
            System.out.println("-----");
            System.out.println("Starting queued request: " + _queuedRequest.description());
            System.out.println("-----");
            handleImmediateRequest(_queuedRequest);
            _queuedRequest = null;
        }
    }

    private void executeActiveRequest() {
        var activeRequest = getActiveRequest();
        if (activeRequest == null) return;

        System.out.println("-----");
        System.out.println("🤖asdfasdf asd " + activeRequest.description());
        System.out.println("-----");

        setSubsystemStates(activeRequest.targetPosition());

        if (activeRequestFinished()) {
            System.out.println("-----");
            System.out.println("✅Request completed: " + activeRequest.description());
            System.out.println("-----");
            _activeRequests.remove();
        }
    }

    private void setSubsystemStates(SuperstructureState state) {
        if (state.getElevator().isPresent()) {
            _container.getElevator().setDesiredState(state.getElevator().get());
        }
        if (state.getCoral().isPresent()) {
            _container.getCoral().setDesiredState(state.getCoral().get());
        }
        if (state.getAlgae().isPresent()) {
            _container.getAlgae().setDesiredState(state.getAlgae().get());
        }
        if (state.getIntake().isPresent()) {
            _container.getIntake().setDesiredState(state.getIntake().get());
        }
    }

    private boolean activeRequestFinished() {
        var goal = getActiveRequest().targetPosition();
        if (goal.getElevator().isPresent()
                && !_container.getElevator().isAtState(goal.getElevator().get())) return false;
        if (goal.getAlgae().isPresent() && !_container.getAlgae().isAtState(goal.getAlgae().get()))
            return false;
        if (goal.getCoral().isPresent() && !_container.getCoral().isAtState(goal.getCoral().get()))
            return false;
        return true;
    }

    private SuperstructureState getCurrentState() {
        return new SuperstructureState(
                Optional.of(_container.getElevator().getCurrentState()),
                Optional.of(_container.getCoral().getCurrentState()),
                Optional.of(_container.getAlgae().getCurrentState()),
                Optional.of(IntakeState.IDLE) // TODO FIXME
                );
    }

    public SuperstructureRequest getActiveRequest() {
        return _activeRequests.peek();
    }

    public SuperstructureRequest getQueuedRequest() {
        return _queuedRequest;
    }

    @Override
    public String getLogBase() {
        return "RequestHandler";
    }
}
