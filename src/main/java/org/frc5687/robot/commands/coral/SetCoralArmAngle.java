package org.frc5687.robot.commands.coral;

import java.util.function.DoubleSupplier;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.coralarm.CoralArmSubsystem;

public class SetCoralArmAngle extends OutliersCommand {

    private final CoralArmSubsystem _arm;
    private DoubleSupplier _angleSupplier;

    public SetCoralArmAngle(CoralArmSubsystem arm, DoubleSupplier angleSupplier) {
        _arm = arm;
        _angleSupplier = angleSupplier;
        addRequirements(_arm);
    }

    @Override
    protected void execute(double timestamp) {
        double targetAngle = Math.atan2(0, _angleSupplier.getAsDouble());
        _arm.setArmAngle(targetAngle);
    }

    @Override
    public boolean isFinished() {
        return _arm.isAtDesiredAngle();
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }
}
