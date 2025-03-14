package org.frc5687.robot.commands.coral;

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
        System.out.println("Starting index command");
        // if (_initialRequest == null) {
        //     _initialRequest = _manager.getRequestHandler().getActiveRequest();
        // }
        _coral.setWheelMotorDutyCycle(0.3);
    }

    @Override
    public void execute(double timestamp) {
        _coral.setWheelMotorDutyCycle(0.3);
    }

    @Override
    public boolean isFinished() {
        if (_coral.isCoralDetected()) {
            System.out.println("Index complete due to seeing coral");
            return true;
        }

        SuperstructureRequest activeRequest = _manager.getRequestHandler().getActiveRequest();

        // check if there's a new active request with a different target position
        // if (_initialRequest == null) {
        //     System.out.println("Request is null");
        //     return true;
        // }

        boolean differentRequest =
                activeRequest != null
                        && !_initialRequest.targetPosition().equals(activeRequest.targetPosition());

        if (differentRequest) {
            System.out.println("Request has changed");
            System.out.println("Initial target position: " + _initialRequest.targetPosition());
            System.out.println("Active target position: " + activeRequest.targetPosition());
        }

        return differentRequest;
    }

    @Override
    public void end(boolean interrupted) {
        if (_coral.isCoralDetected()) {
            double currentPos = _coral.getWheelMotorPosition();
            _coral.setWheelMotorPosition(currentPos + 1.5);
        } else if (!interrupted) {
            // we need to explicitly stop coral
            _coral.setWheelMotorDutyCycle(0);
        }
    }
}
