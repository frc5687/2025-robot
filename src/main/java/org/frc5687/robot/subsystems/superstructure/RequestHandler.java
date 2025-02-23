package org.frc5687.robot.subsystems.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.subsystems.coralarm.CoralState;
import org.frc5687.robot.subsystems.intake.IntakeState;
import org.frc5687.robot.util.EpilogueLog;

public class RequestHandler implements EpilogueLog {
    private final RobotContainer _container;

    private SuperstructureRequest _activeRequest = null;
    private SuperstructureRequest _queuedRequest = null;
    private SuperstructureRequest _pendingRequest = null;
    private boolean _isSafeTransitioning = false;
    private double _transitionStartTime = 0;
    private static final double TRANSITION_TIMEOUT = 3.0;

    public RequestHandler(RobotContainer container) {
        _container = container;
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
            // case AUTO_SEQUENCE:
            //     handleAutoRequest(request);
            //     break;
        }
    }

    private void handleImmediateRequest(SuperstructureRequest request) {
        _queuedRequest = null;

        if (_activeRequest != null) {
            System.out.println("Interrupting active request: " + _activeRequest.description());
            if (needsSafeTransition(_activeRequest.targetPosition(), request.targetPosition())) {
                _pendingRequest = request;
                _isSafeTransitioning = true;
                _transitionStartTime = Timer.getFPGATimestamp();
                setSubsystemStates(SuperstructureGoals.SAFE_CORAL_TRANSITION); 
                updateMotors(MotorState.HOLD);
            } else {
                // Direct transition is safe
                _activeRequest = request;
            }
        } else {
            _activeRequest = request;
        }
    }

    private void handleQueuedRequest(SuperstructureRequest request) {
        _queuedRequest = request;
        System.out.println("Request queued: " + request.description());
    }

    // private void handleAutoRequest(SuperstructureRequest request) {
    //     if (_activeRequest == null || _activeRequest.type() == RequestType.AUTO_SEQUENCE) {
    //         _queuedRequest = request;
    //         System.out.println("Auto sequence queued: " + request.description());
    //     }
    // }

    public void execute() {
        handleSafeTransition();
        if (_isSafeTransitioning) return;

        checkAndStartQueuedRequest();
        executeActiveRequest();
    }

    private void handleSafeTransition() {
        if (_isSafeTransitioning
                && Timer.getFPGATimestamp() - _transitionStartTime > TRANSITION_TIMEOUT) {
            System.out.println("Safe transition timeout, forcing completion");
            _isSafeTransitioning = false;
            if (_pendingRequest != null) {
                _activeRequest = _pendingRequest;
                _pendingRequest = null;
            }
            return;
        }

        if (_isSafeTransitioning && isAtState(SuperstructureGoals.SAFE_CORAL_TRANSITION)) {
            System.out.println("Safe transition complete");
            _isSafeTransitioning = false;
            if (_pendingRequest != null) {
                _activeRequest = _pendingRequest;
                _pendingRequest = null;
            }
        }
    }

    private void checkAndStartQueuedRequest() {
        if (_activeRequest == null && _queuedRequest != null) {
            if (_queuedRequest.driveCondition().get()) {
                System.out.println("Starting queued request: " + _queuedRequest.description());

                SuperstructureState currentState = getCurrentState();
                if (needsSafeTransition(currentState, _queuedRequest.targetPosition())) {
                    _pendingRequest = _queuedRequest;
                    _queuedRequest = null;
                    _isSafeTransitioning = true;
                    _transitionStartTime = Timer.getFPGATimestamp();
                    setSubsystemStates(SuperstructureGoals.SAFE_CORAL_TRANSITION);
                    updateMotors(MotorState.HOLD);
                } else {
                    _activeRequest = _queuedRequest;
                    _queuedRequest = null;
                }
            }
        }
    }

    private void executeActiveRequest() {
        if (_activeRequest == null) return;

        setSubsystemStates(_activeRequest.targetPosition());
        updateMotors(_activeRequest.targetMotors());

        if (isAtState(_activeRequest.targetPosition())) {
            System.out.println("Request completed: " + _activeRequest.description());
            _activeRequest = null;
        }
    }

    private void setSubsystemStates(SuperstructureState state) {
        _container.getElevator().setDesiredState(state.getElevator());
        _container.getCoral().setDesiredState(state.getCoral());
        _container.getAlgae().setDesiredState(state.getAlgae());
        _container.getIntake().setDesiredState(state.getIntake());
    }

    private void updateMotors(MotorState state) {
        _container.getCoral().setWheelMotorDutyCycle(state.coralSpeed());
        _container.getAlgae().setWheelMotorVoltage(state.algaeSpeed() * 12.0);
        _container.getIntake().setRollerVoltage(state.intakeSpeed() * 12.0);
    }

    private boolean needsSafeTransition(SuperstructureState from, SuperstructureState to) {
        if (from == null || to == null) return false;

        double heightDiff = Math.abs(from.getElevator().getHeight() - to.getElevator().getHeight());
        if (heightDiff > 0.1) return true;

        if (from.getCoral() != to.getCoral()) {
            double currentAngle = _container.getCoral().getArmAngleRads();
            double safeThreshold = CoralState.STOWED.getArmAngle() + Units.degreesToRadians(10);
            if (currentAngle > safeThreshold) return true;
        }

        return false;
    }

    private boolean isAtState(SuperstructureState state) {
        return _container.getElevator().isAtState(state.getElevator())
                && _container.getCoral().isAtState(state.getCoral())
                && _container.getAlgae().isAtState(state.getAlgae()); // TODO: INTAKE
    }

    private SuperstructureState getCurrentState() {
        return new SuperstructureState(
                _container.getElevator().getCurrentState(),
                _container.getCoral().getCurrentState(),
                _container.getAlgae().getCurrentState(),
                IntakeState.IDLE // TODO FIXME
                );
    }

    public SuperstructureRequest getActiveRequest() {
        return _activeRequest;
    }

    public SuperstructureRequest getQueuedRequest() {
        return _queuedRequest;
    }

    public SuperstructureRequest getPendingRequest() {
        return _pendingRequest;
    }

    public boolean isTransitioning() {
        return _isSafeTransitioning;
    }

    @Override
    public String getLogBase() {
        return "RequestHandler";
    }
}
