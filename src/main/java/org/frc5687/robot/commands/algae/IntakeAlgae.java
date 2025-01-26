package org.frc5687.robot.commands.algae;

import java.util.function.DoubleSupplier;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.algaearm.AlgaeArmSubsystem;

public class IntakeAlgae extends OutliersCommand {

    AlgaeArmSubsystem _algaeArm;
    double _angle;
    double _voltage;
    DoubleSupplier _angleSupplier;

    public IntakeAlgae(
            AlgaeArmSubsystem algaeArmSubsystem, double voltage, DoubleSupplier angleSupplier) {
        _algaeArm = algaeArmSubsystem;
        _voltage = voltage;
        _angleSupplier = angleSupplier;

        addRequirements(_algaeArm);
    }

    @Override
    protected void execute(double timestamp) {
        double targetAngle = Math.atan2(0, _angleSupplier.getAsDouble());
        _algaeArm.setArmAngle(targetAngle);
        _algaeArm.setAlgaeMotorVoltage(_voltage);
    }

    @Override
    public void end(boolean interrupted) {
        _algaeArm.setAlgaeMotorVoltage(0);
    }
}
