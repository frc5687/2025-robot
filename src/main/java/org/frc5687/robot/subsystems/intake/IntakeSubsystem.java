package org.frc5687.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.subsystems.OutliersSubsystem;
import org.frc5687.robot.subsystems.SubsystemIO;

public class IntakeSubsystem extends OutliersSubsystem<IntakeInputs, IntakeOutputs> {

    public IntakeSubsystem(RobotContainer container, SubsystemIO<IntakeInputs, IntakeOutputs> io) {
        super(container, io, new IntakeInputs(), new IntakeOutputs());
    }

    @Override
    protected void processInputs() {
        RobotStateManager.getInstance().updateIntakeArm(_inputs.armAngleRads);
    }

    @Override
    protected void periodic(IntakeInputs inputs, IntakeOutputs outputs) {
        _outputs.dynamicsFF = calculateGravityFeedForward(getPivotArmAngleRads());
    }

    public void setDesiredPivotAngle(IntakeState state) {
        setDesiredPivotAngle(state.getValue());
    }

    public void setDesiredPivotAngle(double angle) {
        double desiredAngleClamped =
                MathUtil.clamp(angle, Constants.Intake.MIN_ANGLE, Constants.Intake.MAX_ANGLE);
        _outputs.desiredAngleRad = desiredAngleClamped;
    }

    public void setVoltages(double rollerVoltage, double intakeVoltage) {
        setRollerVoltage(rollerVoltage);
        setIntakeVoltage(intakeVoltage);
    }

    public void setRollerVoltage(double voltage) {
        _outputs.rollerVoltage = voltage;
    }

    public void setIntakeVoltage(double voltage) {
        _outputs.intakeVoltage = voltage;
    }

    public double getPivotArmAngleRads() {
        return _inputs.armAngleRads;
    }

    public boolean isAtDesiredAngle() {
        return Math.abs(
                        new Rotation2d(_inputs.armAngleRads)
                                .minus(new Rotation2d(_outputs.desiredAngleRad))
                                .getDegrees())
                < 1.0;
        // return true;
    }

    public boolean isAtState(IntakeState state) {
        double angleDiff = Math.abs(state.getValue() - getPivotArmAngleRads());
        boolean isWithinPositionTolerance = angleDiff < Units.degreesToRadians(1.0);
        return isWithinPositionTolerance;
        // return true;
    }

    private double calculateGravityFeedForward(double angle) {
        return ((Constants.Intake.ARM_LENGTH / 2.0)
                        * (Constants.Intake.GEARBOX.rOhms * Constants.Intake.ARM_MASS * 9.81)
                        / (Constants.Intake.GEAR_RATIO * Constants.Intake.GEARBOX.KtNMPerAmp))
                * Math.cos(angle);
    }

    public boolean isIntakeCoralDetected() {
        return _inputs.isCoralDetected;
    }
}
