package org.frc5687.robot.commands.superstructure;

import edu.wpi.first.wpilibj.Timer;
import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.algaearm.AlgaeArmSubsystem;
import org.frc5687.robot.subsystems.algaearm.AlgaeState;
import org.frc5687.robot.subsystems.coralarm.CoralArmSubsystem;
import org.frc5687.robot.subsystems.elevator.ElevatorSubsystem;

public class SuperstructurePlace extends OutliersCommand {
    private final ElevatorSubsystem _elevator;
    private final CoralArmSubsystem _coral;
    private final AlgaeArmSubsystem _algae;

    private boolean _timerLock;
    private double _noCoralStartTime;

    public SuperstructurePlace(
            ElevatorSubsystem elevator, CoralArmSubsystem coral, AlgaeArmSubsystem algae) {
        _elevator = elevator;
        _coral = coral;
        _algae = algae;
        addRequirements(_elevator, _coral, _algae);
    }

    @Override
    public void initialize() {
        super.initialize();
        _coral.setWheelMotorDutyCycle(Constants.CoralArm.WHEEL_EJECT_CORAL_DUTY_CYCLE);
        _algae.setDesiredState(AlgaeState.IDLE);
        _noCoralStartTime = Timer.getFPGATimestamp();
        _timerLock = false;
    }

    @Override
    protected void execute(double timestamp) {

        if (!_coral.isCoralDetected() && !_timerLock) {
            _timerLock = true;
            _noCoralStartTime = Timer.getFPGATimestamp();
        } else if (_coral.isCoralDetected()) {
            _timerLock = false;
        }
    }

    @Override
    public boolean isFinished() {
        boolean expiredTime = (Timer.getFPGATimestamp() > (_noCoralStartTime + 0.4));
        return expiredTime && _algae.isAtDesiredAngle();
    }

    @Override
    public void end(boolean interrupted) {
        _coral.setWheelMotorPosition(_coral.getWheelMotorPosition());
        _elevator.mapToClosestState();
        super.end(interrupted);
    }
}
