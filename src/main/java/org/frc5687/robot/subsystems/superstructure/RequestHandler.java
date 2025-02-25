package org.frc5687.robot.subsystems.superstructure;

import java.util.ArrayDeque;
import java.util.Optional;
import java.util.Queue;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.subsystems.coralarm.CoralState;
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
        var currentElevatorHeight = _container.getElevator().getInputs().heightPositionMeters;
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
            _container.getElevator().setDesiredHeight(state.getElevator().get());
        }
        if (state.getCoral().isPresent()) {
            _container.getCoral().setArmAngle(state.getCoral().get());
        }
        if (state.getAlgae().isPresent()) {
            _container.getAlgae().setArmAngle(state.getAlgae().get());
        }
        if (state.getIntake().isPresent()) {
            _container.getIntake().setDesiredPivotAngle(state.getIntake().get());
        }
    }

    private boolean activeRequestFinished() {
        var goal = getActiveRequest().targetPosition();
        if (goal.getElevator().isPresent()
                && !_container.getElevator().isAtState(goal.getElevator().get())) {
            System.out.println("elevator not finished");
            return false;
        }
        ;
        if (goal.getAlgae().isPresent() && !_container.getAlgae().isAtState(goal.getAlgae().get())) {
            System.out.println("algae not finished");
            return false;
        }
        if (goal.getCoral().isPresent() && !_container.getCoral().isAtState(goal.getCoral().get())) {
            System.out.println("coral not finished");
            return false;
        }
        System.out.println("all finished");
        return true;
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
