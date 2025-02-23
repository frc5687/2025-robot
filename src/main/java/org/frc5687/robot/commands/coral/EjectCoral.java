package org.frc5687.robot.commands.coral;

import edu.wpi.first.wpilibj.Timer;
import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.coralarm.CoralArmSubsystem;

public class EjectCoral extends OutliersCommand {
    private final CoralArmSubsystem _coral;
    private double _startTime;

    public EjectCoral(CoralArmSubsystem coral) {
        _coral = coral;
        addRequirements(_coral);
    }

    @Override
    public void initialize() {
        _startTime = Timer.getFPGATimestamp();
        _coral.setWheelMotorDutyCycle(Constants.CoralArm.WHEEL_EJECT_CORAL_DUTY_CYCLE);
    }

    @Override
    protected void execute(double timestamp) {}

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - _startTime) > Constants.CoralArm.EJECT_TIME;
    }
}
