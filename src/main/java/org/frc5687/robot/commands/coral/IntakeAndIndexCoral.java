package org.frc5687.robot.commands.coral;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.coralarm.CoralArmSubsystem;
import org.frc5687.robot.subsystems.superstructure.SuperstructureManager;
import org.frc5687.robot.subsystems.superstructure.SuperstructureRequest;

public class IntakeAndIndexCoral extends OutliersCommand {
    private final CoralArmSubsystem _coral;
    private final SuperstructureManager _manager;

    private static boolean _isRunning = false;

    private enum State {
        INITIALIZE,
        PREPARE_FOR_TRANSFER,
        TRANSFERRING_CORAL,
        COMPLETING_TRANSFER,
        COMPLETED
    }

    private State _currentState = State.INITIALIZE;
    private SuperstructureRequest _initialRequest;
    private final Timer _stateTimer = new Timer();
    private boolean _coralDetected = false;

    public IntakeAndIndexCoral(
            CoralArmSubsystem coral,
            SuperstructureManager manager,
            SuperstructureRequest initialRequest) {
        _coral = coral;
        _manager = manager;
        _initialRequest = initialRequest;
        addRequirements(_coral);
    }

    @Override
    public void initialize() {
        if (_isRunning) {
            System.out.println("GroundIndexCoral already running, canceling this instance");
            CommandScheduler.getInstance().cancel(this);
            return;
        }

        System.out.println("Starting index state machine");
        _isRunning = true;
        _currentState = State.INITIALIZE;
        _stateTimer.reset();
        _stateTimer.start();
        _coralDetected = false;

        if (_initialRequest == null) {
            SuperstructureRequest activeRequest = _manager.getRequestHandler().getActiveRequest();
            if (activeRequest != null) {
                _initialRequest = activeRequest;
            } else {
                _initialRequest = _manager.getRequestHandler().getLastActiveRequest();
            }
        }
    }

    @Override
    public void execute(double timestamp) {
        if (!_coralDetected && _coral.isCoralDetected()) {
            _coralDetected = true;
            double currentPos = _coral.getWheelMotorPosition();
            _coral.setWheelMotorPosition(currentPos + Constants.CoralArm.WHEEL_INDEX_ROTATIONS);
            System.out.println("Coral detected - Adding index rotations");
        }

        switch (_currentState) {
            case INITIALIZE:
                if (_manager.getRequestHandler().getActiveRequest() == null) {
                    _currentState = State.PREPARE_FOR_TRANSFER;
                    System.out.println("State: INITIALIZE -> PREPARE_FOR_TRANSFER");
                    break;
                }
                break;

            case PREPARE_FOR_TRANSFER:
                if (_coral.isAtState(Constants.SuperstructureGoals.RECEIVE_FROM_FUNNEL.getCoral().get())) {
                    _coral.setWheelMotorDutyCycle(Constants.CoralArm.WHEEL_FUNNEL_INDEX_DUTY_CYCLE);
                    _currentState = State.TRANSFERRING_CORAL;
                    System.out.println("State: PREPARE_FOR_TRANSFER -> TRANSFERRING_CORAL");
                }
                break;

            case TRANSFERRING_CORAL:
                if (_coralDetected) {
                    _currentState = State.COMPLETING_TRANSFER;
                    System.out.println("State: TRANSFERRING_CORAL -> COMPLETING_TRANSFER");
                }
                break;

            case COMPLETING_TRANSFER:
                _currentState = State.COMPLETED;
                System.out.println("State: TRANSFERING_CORAL -> COMPLETED");
                break;

            case COMPLETED:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        SuperstructureRequest activeRequest = _manager.getRequestHandler().getActiveRequest();
        SuperstructureRequest lastRequest = _manager.getRequestHandler().getLastActiveRequest();

        if (_initialRequest != null) {
            boolean differentFromActiveRequest =
                    activeRequest != null
                            && !_initialRequest.equals(activeRequest)
                            && !_initialRequest.targetPosition().equals(activeRequest.targetPosition());

            boolean differentFromLastRequest =
                    lastRequest != null
                            && lastRequest != _initialRequest
                            && !_initialRequest.targetPosition().equals(lastRequest.targetPosition());

            boolean requestChanged = differentFromActiveRequest || differentFromLastRequest;

            if (requestChanged) {
                System.out.println("Request has changed, ending index command");
                if (activeRequest != null) {
                    System.out.println("Initial Request = " + activeRequest.description());
                } else {
                    System.out.println("active request is null");
                }
                if (lastRequest != null) {
                    System.out.println("Initial Request = " + lastRequest.description());

                } else {
                    System.out.println("last request is null");
                }
                return true;
            }
        }

        return _currentState == State.COMPLETED;
    }

    @Override
    public void end(boolean interrupted) {
        _isRunning = false;

        if (interrupted) {
            System.out.println("IntakeAndIndexCoral interrupted");
            if (!_coral.isCoralDetected()) {
                _coral.setWheelMotorDutyCycle(0);
            }
        } else {
            System.out.println("IntakeAndIndexCoral completed successfully");
            if (!_coralDetected) {
                _coral.setWheelMotorDutyCycle(0);
            }
        }
    }

    public static boolean isRunning() {
        return _isRunning;
    }
}
