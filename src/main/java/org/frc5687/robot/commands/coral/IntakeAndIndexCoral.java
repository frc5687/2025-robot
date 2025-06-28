package org.frc5687.robot.commands.coral;

import edu.wpi.first.wpilibj.Timer;
import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.coralarm.CoralArmSubsystem;
import org.frc5687.robot.subsystems.superstructure.SuperstructureManager;
import org.frc5687.robot.subsystems.superstructure.SuperstructureRequest;

public class IntakeAndIndexCoral extends OutliersCommand {
    private final CoralArmSubsystem _coral;
    private final SuperstructureManager _manager;
    private SuperstructureRequest _initialRequest;

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
        double timer1 = Timer.getFPGATimestamp();
        System.out.println("Starting index command");
        if (_initialRequest == null) {
            SuperstructureRequest activeRequest = _manager.getRequestHandler().getActiveRequest();
            if (activeRequest != null) {
                _initialRequest = activeRequest;
            } else {
                _initialRequest = _manager.getRequestHandler().getLastActiveRequest();
            }
        }
        _coral.setWheelMotorDutyCycle(Constants.CoralArm.WHEEL_FUNNEL_INDEX_DUTY_CYCLE);
        System.out.println("init took " + (Timer.getFPGATimestamp() - timer1) + " seconds");
    }

    @Override
    public void execute(double timestamp) {
        _coral.setWheelMotorDutyCycle(Constants.CoralArm.WHEEL_FUNNEL_INDEX_DUTY_CYCLE);
    }

    @Override
    public boolean isFinished() {
        if (_coral.isCoralDetected()) {
            System.out.println("Index complete due to seeing coral");
            return true;
        }

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

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (_coral.isCoralDetected()) {
            double currentPos = _coral.getWheelMotorPosition();
            _coral.setWheelMotorPosition(currentPos + Constants.CoralArm.WHEEL_INDEX_ROTATIONS);
        } else if (!interrupted) {
            // we need to explicitly stop coral
            _coral.setWheelMotorDutyCycle(0);
        }
    }
}
