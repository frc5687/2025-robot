package org.frc5687.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.subsystems.OutliersSubsystem;
import org.frc5687.robot.subsystems.SubsystemIO;

public class IntakeSubsystem extends OutliersSubsystem<IntakeInputs, IntakeOutputs> {

    private static final double POSITION_TOLERANCE_RAD = Units.degreesToRadians(1.0);

    public IntakeSubsystem(RobotContainer container, SubsystemIO<IntakeInputs, IntakeOutputs> io) {
        super(container, io, new IntakeInputs(), new IntakeOutputs());
        _outputs.desiredAngleRad = IntakeState.IDLE.getValue();
    }

    @Override
    protected void processInputs() {
        RobotStateManager.getInstance().updateIntakeArm(_inputs.armAngleRads);
    }

    @Override
    protected void periodic(IntakeInputs inputs, IntakeOutputs outputs) {
        outputs.dynamicsFF = calculateGravityFeedForward(getPivotArmAngleRads());
    }

    public void setDesiredPivotAngle(IntakeState state) {
        setDesiredPivotAngle(state.getValue());
    }

    public void setDesiredPivotAngle(double angle) {
        double desiredAngleClamped =
                MathUtil.clamp(angle, Constants.Intake.MIN_ANGLE, Constants.Intake.MAX_ANGLE);

        _outputs.desiredAngleRad = desiredAngleClamped;
    }

    public void setVoltages(double intakeVoltage) {
        setIntakeVoltage(intakeVoltage);
    }

    public void setIntakeVoltage(double voltage) {
        _outputs.intakeVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
    }

    public double getPivotArmAngleRads() {
        return _inputs.armAngleRads;
    }

    public double getBeltVelocity() {
        return _inputs.beltVelocity;
    }

    public double getBeltAmps() {
        return _inputs.rollerCurrent;
    }

    public boolean isAtDesiredAngle() {
        double positionError = Math.abs(_inputs.armAngleRads - _outputs.desiredAngleRad);
        return positionError < POSITION_TOLERANCE_RAD;
    }

    public boolean isAtState(IntakeState state) {
        double angleDiff = Math.abs(state.getValue() - getPivotArmAngleRads());
        return angleDiff < POSITION_TOLERANCE_RAD;
    }

    public boolean isClimberOutForClimb() {
        return _outputs.desiredAngleRad == IntakeState.CLIMB.getValue();
    }

    private double calculateGravityFeedForward(double angle) {
        // return ((Constants.Intake.ARM_LENGTH / 2.0)
        //                 * (Constants.Intake.GEARBOX.rOhms * Constants.Intake.ARM_MASS * 9.81)
        //                 / (Constants.Intake.GEAR_RATIO * Constants.Intake.GEARBOX.KtNMPerAmp))
        //         * Math.cos(angle);
        return 0.0;
    }

    public void disableIntakeMotors() {
        _outputs.disabled = true;
    }

    public boolean isIntakeCoralDetected() {
        return _inputs.isCoralDetected;
    }
}
