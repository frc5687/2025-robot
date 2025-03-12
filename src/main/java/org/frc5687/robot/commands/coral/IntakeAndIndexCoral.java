package org.frc5687.robot.commands.coral;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.coralarm.CoralArmSubsystem;
import org.frc5687.robot.subsystems.superstructure.SuperstructureManager;
import org.frc5687.robot.subsystems.superstructure.SuperstructureRequest;

public class IntakeAndIndexCoral extends OutliersCommand {
    private final CoralArmSubsystem _coral;
    private final SuperstructureManager _manager;
    private final SuperstructureRequest _initialRequest;

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
        _coral.setWheelMotorDutyCycle(0.3);
    }

    @Override
    public void execute(double timestamp) {
        _coral.setWheelMotorDutyCycle(0.3);
    }

    @Override
    public boolean isFinished() {
        if (_coral.isCoralDetected()) {
            return true;
        }

        // We need to check if the active (which should always be receive from funnel) is the same
        SuperstructureRequest activeRequest = _manager.getRequestHandler().getActiveRequest();
        return activeRequest != null && activeRequest != _initialRequest;
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
