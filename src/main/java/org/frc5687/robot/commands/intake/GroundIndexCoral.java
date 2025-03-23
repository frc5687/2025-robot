package org.frc5687.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.coralarm.CoralArmSubsystem;
import org.frc5687.robot.subsystems.intake.IntakeState;
import org.frc5687.robot.subsystems.intake.IntakeSubsystem;
import org.frc5687.robot.subsystems.superstructure.RequestType;
import org.frc5687.robot.subsystems.superstructure.SuperstructureManager;
import org.frc5687.robot.subsystems.superstructure.SuperstructureRequest;

public class GroundIndexCoral extends OutliersCommand {
    private final CoralArmSubsystem _coral;
    private final IntakeSubsystem _intake;
    private final SuperstructureManager _manager;

    private static boolean _isRunning = false;

    private enum State {
        INITIALIZE,
        PREPARE_FOR_TRANSFER,
        TRANSFERRING_CORAL,
        COMPLETING_TRANSFER,
        STOW_INTAKE,
        COMPLETED
    }

    private State _currentState = State.INITIALIZE;
    private SuperstructureRequest _initialRequest;
    private final Timer _stateTimer = new Timer();
    private boolean _coralDetected = false;

    public GroundIndexCoral(
            CoralArmSubsystem coral,
            IntakeSubsystem intake,
            SuperstructureManager manager,
            SuperstructureRequest initialRequest) {
        _coral = coral;
        _intake = intake;
        _manager = manager;
        _initialRequest = initialRequest;
        addRequirements(_coral, _intake);
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
                _intake.setVoltages(2.0);
                _manager.receiveFromGroundIntake(RequestType.IMMEDIATE).schedule();
                _currentState = State.PREPARE_FOR_TRANSFER;
                _stateTimer.reset();
                System.out.println("State: INITIALIZE -> PREPARE_FOR_TRANSFER");
                break;

            case PREPARE_FOR_TRANSFER:
                if ((_intake.isAtState(
                                        Constants.SuperstructureGoals.RECEIVE_FROM_GROUND_INTAKE.getIntake().get())
                                || _stateTimer.hasElapsed(1.5))
                        && _stateTimer.hasElapsed(Constants.Intake.INTAKE_PASSOFF_DELAY)) {
                    _coral.setWheelMotorDutyCycle(0.6);
                    _intake.setVoltages(-3.0);
                    _currentState = State.TRANSFERRING_CORAL;
                    _stateTimer.reset();
                    System.out.println("State: PREPARE_FOR_TRANSFER -> TRANSFERRING_CORAL");
                }
                break;

            case TRANSFERRING_CORAL:
                if (_coralDetected) {
                    _currentState = State.COMPLETING_TRANSFER;
                    _stateTimer.reset();
                    System.out.println("State: TRANSFERRING_CORAL -> COMPLETING_TRANSFER");
                } else if (_stateTimer.hasElapsed(2.0)) {
                    _currentState = State.COMPLETING_TRANSFER;
                    _stateTimer.reset();
                    System.out.println("State: TRANSFERRING_CORAL -> COMPLETING_TRANSFER (timeout)");
                }
                break;

            case COMPLETING_TRANSFER:
                if (_stateTimer.hasElapsed(0.5)) {
                    _intake.setVoltages(0);
                    _currentState = State.STOW_INTAKE;
                    _stateTimer.reset();
                    System.out.println("State: COMPLETING_TRANSFER -> STOW_INTAKE");
                }
                break;

            case STOW_INTAKE:
                _intake.setDesiredPivotAngle(IntakeState.IDLE);
                _currentState = State.COMPLETED;
                System.out.println("State: STOW_INTAKE -> COMPLETED");
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
                return true;
            }
        }

        return _currentState == State.COMPLETED;
    }

    @Override
    public void end(boolean interrupted) {
        _isRunning = false;

        if (interrupted) {
            System.out.println("GroundIndexCoral interrupted");
            _intake.setVoltages(0);
            if (!_coral.isCoralDetected()) {
                _coral.setWheelMotorDutyCycle(0);
            }
        } else {
            System.out.println("GroundIndexCoral completed successfully");
            _intake.setVoltages(0);

            if (!_coralDetected) {
                _coral.setWheelMotorDutyCycle(0);
            }
        }
    }

    public static boolean isRunning() {
        return _isRunning;
    }
}
