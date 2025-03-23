package org.frc5687.robot.subsystems.superstructure;

import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
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
    private SuperstructureRequest _lastActiveRequest = null;

    private static final double ELEVATOR_THRESHOLD = 0.3; // L3
    private static final double CORAL_ANGLE_THRESHOLD = Units.degreesToRadians(250);

    private boolean _elevatorMovementStarted = false;
    private boolean _coralMovementStarted = false;
    private SuperstructureState _finalTargetState = null;
    private boolean _inParallelMovement = false;
    private boolean _movingUp = false;

    public RequestHandler(RobotContainer container) {
        _container = container;
        _activeRequests = new ArrayDeque<>();
        _queuedRequest = null;
    }

    public void handleNewRequest(SuperstructureRequest request) {
        // log("NewRequest", request.description());

        switch (request.type()) {
            case IMMEDIATE:
                handleImmediateRequest(request);
                break;
            case QUEUED:
                handleQueuedRequest(request);
                break;
            case AUTO_SEQUENCE:
                handleQueuedRequest(request);
                break;
        }
    }

    private void handleImmediateRequest(SuperstructureRequest request) {
        _queuedRequest = null;
        _activeRequests.clear();

        _elevatorMovementStarted = false;
        _coralMovementStarted = false;
        _finalTargetState = null;
        _inParallelMovement = false;

        // movement that crosses the collision zone
        if (needsCollisionAvoidance(request)) {
            _finalTargetState = request.targetPosition();
            _lastActiveRequest = request;
            _inParallelMovement = true;

            double currentElevatorHeight = _container.getElevator().getElevatorHeight();
            double targetElevatorHeight = _finalTargetState.getElevator().get().getHeight();
            _movingUp = targetElevatorHeight > currentElevatorHeight;

            addRequest(
                    new SuperstructureRequest(
                            new SuperstructureState(
                                    Optional.empty(), Optional.empty(), Optional.empty(), Optional.empty()),
                            RequestType.IMMEDIATE,
                            request.driveCondition(),
                            "Parallel movement control"));
        } else {
            addRequest(request);
        }
    }

    private boolean needsCollisionAvoidance(SuperstructureRequest request) {
        var elevatorRequest = request.targetPosition().getElevator();
        if (elevatorRequest.isEmpty()) return false;

        double requestedElevatorHeight = elevatorRequest.get().getHeight();
        double currentElevatorHeight = _container.getElevator().getElevatorHeight();

        // Check if we cross L3
        boolean elevatorCrossingThreshold =
                (currentElevatorHeight < ELEVATOR_THRESHOLD && requestedElevatorHeight > ELEVATOR_THRESHOLD)
                        || (currentElevatorHeight > ELEVATOR_THRESHOLD
                                && requestedElevatorHeight < ELEVATOR_THRESHOLD);

        if (!elevatorCrossingThreshold) return false;

        // only move coral if elevator is crossing threshold
        var coralRequest = request.targetPosition().getCoral();
        if (coralRequest.isEmpty()) return false;

        double requestedCoralAngle = coralRequest.get().getArmAngle();
        double currentCoralAngle = _container.getCoral().getArmAngleRads();

        // See if coral movement will cross the threshold angle
        boolean coralCrossingThreshold =
                (currentCoralAngle < CORAL_ANGLE_THRESHOLD && requestedCoralAngle > CORAL_ANGLE_THRESHOLD)
                        || (currentCoralAngle > CORAL_ANGLE_THRESHOLD
                                && requestedCoralAngle < CORAL_ANGLE_THRESHOLD)
                        ||
                        // check if coral is already past collision and needs to move
                        (currentCoralAngle > CORAL_ANGLE_THRESHOLD && requestedCoralAngle != currentCoralAngle);

        return elevatorCrossingThreshold && coralCrossingThreshold;
    }

    private void addRequest(SuperstructureRequest request) {
        // log("AddRequest", request.description());
        _activeRequests.add(request);
    }

    private void handleQueuedRequest(SuperstructureRequest request) {
        _queuedRequest = request;
        // log("RequestQueued", request.description());
    }

    public void execute() {
        log(
                "Previous Active",
                getLastActiveRequest() == null ? "Null" : getLastActiveRequest().description(),
                Importance.CRITICAL);
        log(
                "Active Request",
                getActiveRequest() == null ? "Null" : getActiveRequest().description(),
                Importance.CRITICAL);
        log(
                "Queued Request",
                getQueuedRequest() == null ? "Null" : getQueuedRequest().description(),
                Importance.CRITICAL);
        checkAndStartQueuedRequest();

        if (_inParallelMovement && _finalTargetState != null) {
            handleParallelMovement();
        } else {
            executeActiveRequest();
        }
    }

    private void handleParallelMovement() {
        if (_movingUp) {
            handleParallelMovementUp();
        } else {
            handleParallelMovementDown();
        }

        if (isParallelMovementComplete()) {
            // This is done in the check for collision as the request is 
            // _lastActiveRequest = getActiveRequest();

            _activeRequests.remove();
            _inParallelMovement = false;
            _finalTargetState = null;
        }
    }

    private void handleParallelMovementUp() {
        double currentCoralAngle = _container.getCoral().getArmAngleRads();

        // Step 1: Start moving coral arm to safe position if not started
        if (!_coralMovementStarted) {
            _container.getCoral().setArmAngle(CoralState.PLACING_L4);
            _coralMovementStarted = true;
            // log("ParallelMovementUp", "Started coral arm movement to safe position");
        }

        // Step 2: As soon as coral angle crosses threshold, start elevator
        if (_coralMovementStarted
                && !_elevatorMovementStarted
                && currentCoralAngle >= CORAL_ANGLE_THRESHOLD) {
            if (_finalTargetState.getElevator().isPresent()) {
                _container.getElevator().setDesiredHeight(_finalTargetState.getElevator().get());
                _elevatorMovementStarted = true;
                // log("ParallelMovementUp", "Coral arm crossed threshold, starting elevator movement");
            }
        }

        // Step 3: Once elevator is past threshold, move coral to final position if needed
        double currentElevatorHeight = _container.getElevator().getElevatorHeight();
        if (_elevatorMovementStarted && currentElevatorHeight > ELEVATOR_THRESHOLD) {
            if (_finalTargetState.getCoral().isPresent()
                    && !_finalTargetState.getCoral().get().equals(CoralState.PLACING_L4)) {
                _container.getCoral().setArmAngle(_finalTargetState.getCoral().get());
                // log("ParallelMovementUp", "Elevator crossed threshold, moving coral to final position");
            }

            if (_finalTargetState.getAlgae().isPresent()) {
                _container.getAlgae().setArmAngle(_finalTargetState.getAlgae().get());
            }
            if (_finalTargetState.getIntake().isPresent()) {
                _container.getIntake().setDesiredPivotAngle(_finalTargetState.getIntake().get());
            }
        }
    }

    private void handleParallelMovementDown() {
        if (!_elevatorMovementStarted) {
            if (_finalTargetState.getElevator().isPresent()) {
                _container.getElevator().setDesiredHeight(_finalTargetState.getElevator().get());
                _elevatorMovementStarted = true;
                // log("ParallelMovementDown", "Started elevator down movement");
            }
        }

        // Step 2: As soon as elevator crosses threshold, move coral
        double currentElevatorHeight = _container.getElevator().getElevatorHeight();
        if (_elevatorMovementStarted
                && !_coralMovementStarted
                && currentElevatorHeight < ELEVATOR_THRESHOLD) {
            if (_finalTargetState.getCoral().isPresent()) {
                _container.getCoral().setArmAngle(_finalTargetState.getCoral().get());
                _coralMovementStarted = true;
                // log(
                //         "ParallelMovementDown",
                //         "Elevator crossed below threshold, moving coral to final position");
            }

            if (_finalTargetState.getAlgae().isPresent()) {
                _container.getAlgae().setArmAngle(_finalTargetState.getAlgae().get());
            }
            if (_finalTargetState.getIntake().isPresent()) {
                _container.getIntake().setDesiredPivotAngle(_finalTargetState.getIntake().get());
            }
        }
    }

    private boolean isParallelMovementComplete() {
        if (_finalTargetState == null) return true;

        boolean elevatorDone =
                !_finalTargetState.getElevator().isPresent()
                        || _container.getElevator().isAtState(_finalTargetState.getElevator().get());

        boolean coralDone =
                !_finalTargetState.getCoral().isPresent()
                        || _container.getCoral().isAtState(_finalTargetState.getCoral().get());

        boolean algaeDone =
                !_finalTargetState.getAlgae().isPresent()
                        || _container.getAlgae().isAtState(_finalTargetState.getAlgae().get());

        boolean intakeDone =
                !_finalTargetState.getIntake().isPresent()
                        || _container.getIntake().isAtState(_finalTargetState.getIntake().get());

        return elevatorDone && coralDone && algaeDone && intakeDone;
    }

    private void checkAndStartQueuedRequest() {
        if (_queuedRequest != null && _queuedRequest.driveCondition().get()) {
            log(
                    "Request Start Pose",
                    _container.getDrive().getPose(),
                    Pose2d.struct,
                    Importance.CRITICAL);
            // log("StartingQueuedRequest", _queuedRequest.description());
            handleImmediateRequest(_queuedRequest);
            _queuedRequest = null;
        }
    }

    private void executeActiveRequest() {
        var activeRequest = getActiveRequest();
        if (activeRequest == null) {
            return;
        }

        _lastActiveRequest = activeRequest;
        setSubsystemStates(activeRequest.targetPosition());

        if (activeRequestFinished()) {
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
            log("RequestProgress", "elevator not finished");
            return false;
        }

        if (goal.getAlgae().isPresent() && !_container.getAlgae().isAtState(goal.getAlgae().get())) {
            log("RequestProgress", "algae not finished");
            return false;
        }

        if (goal.getCoral().isPresent() && !_container.getCoral().isAtState(goal.getCoral().get())) {
            log("RequestProgress", "coral not finished");
            return false;
        }

        if (goal.getIntake().isPresent() && !_container.getIntake().isAtState(goal.getIntake().get())) {
            log("RequestProgress", "intake not finished");
            return false;
        }

        // log("RequestProgress", "all finished");
        return true;
    }

    public SuperstructureRequest getActiveRequest() {
        return _activeRequests.peek();
    }

    public SuperstructureRequest getQueuedRequest() {
        return _queuedRequest;
    }

    public SuperstructureRequest getLastActiveRequest() {
        return _lastActiveRequest;
    }

    @Override
    public String getLogBase() {
        return "RequestHandler";
    }
}
