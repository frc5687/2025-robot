package org.frc5687.robot.commands.coral;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.coralarm.CoralArmSubsystem;

public class SetCoralArmAngle extends OutliersCommand {

    private final CoralArmSubsystem _arm;
    private final double _angleRad;

    public SetCoralArmAngle(CoralArmSubsystem arm, double angleRad) {
        _arm = arm;
        _angleRad = angleRad;
        addRequirements(_arm);
    }

    @Override
    public void initialize() {
        super.initialize();
        System.out.println("Coral setpoint is now: " + _angleRad);
        _arm.setDesiredAngleRadians(_angleRad);
    }

    @Override
    protected void execute(double timestamp) {}

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
