package org.frc5687.robot.commands.elevator;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.elevator.ElevatorState;
import org.frc5687.robot.subsystems.elevator.ElevatorSubsystem;

public class HomeElevator extends OutliersCommand {
    private final ElevatorSubsystem _elevator;
    private final double HOMING_VOLTAGE = -0.3;
    private final double CURRENT_THRESHOLD = 25.0;
    private final double LASER_TARGET = 0.05;
    private final double LASER_TOLERANCE = 0.01;

    private boolean _isFinished = false;

    public HomeElevator(ElevatorSubsystem elevator) {
        _elevator = elevator;
        addRequirements(_elevator);
    }

    @Override
    public void initialize() {
        _isFinished = false;
    }

    @Override
    public void execute(double timestamp) {
        _elevator.setVoltage(HOMING_VOLTAGE);
        boolean currentSpiked = _elevator.getAverageCurrent() > CURRENT_THRESHOLD;

        // Check if laser range finder is at target (you'll need to add laser getter to
        // ElevatorSubsystem)
        boolean atLaserTarget = Math.abs(_elevator.getLaserDistance() - LASER_TARGET) < LASER_TOLERANCE;

        if (currentSpiked || atLaserTarget) {
            _elevator.setVoltage(0.0);
            // Reset encoder position to zero (you'll need to add this method to ElevatorSubsystem)
            // _elevator.resetEncoderPosition();
            _elevator.setDesiredPlatformHeightWorld(ElevatorState.STOWED);
            _isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return _isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        _elevator.setVoltage(0.0);
    }
}
