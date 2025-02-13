package org.frc5687.robot.commands.coral;

import edu.wpi.first.math.util.Units;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.coralarm.CoralArmSubsystem;

public class EjectCoral extends OutliersCommand {
    private final CoralArmSubsystem _coral;

    public EjectCoral(CoralArmSubsystem coral) {
        _coral = coral;
        addRequirements(_coral);
    }

    @Override
    public void initialize() {
        super.initialize();
        _coral.setDesiredAngleRadians(_coral.getArmAngleRads() + Units.degreesToRadians(40));
        _coral.setWheelVoltageCommand(-12);
    }

    @Override
    protected void execute(double timestamp) {}

    @Override
    public boolean isFinished() {
        return _coral.isAtDesiredAngle();
    }

    @Override
    public void end(boolean interrupted) {
        _coral.mapToClosestState();
        super.end(interrupted);
    }
}
