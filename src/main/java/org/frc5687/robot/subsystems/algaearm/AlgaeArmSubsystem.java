package org.frc5687.robot.subsystems.algaearm;

import edu.wpi.first.math.util.Units;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.RobotStateManager.RobotCoordinate;
import org.frc5687.robot.subsystems.OutliersSubsystem;
import org.frc5687.robot.util.TunableDouble;

public class AlgaeArmSubsystem extends OutliersSubsystem<AlgaeInputs, AlgaeOutputs> {

    private final RobotStateManager _stateManger = RobotStateManager.getInstance();
    private TunableDouble algaeP = new TunableDouble("Algae", "kP", Constants.AlgaeArm.kP);
    private TunableDouble algaeI = new TunableDouble("Algae", "kI", Constants.AlgaeArm.kI);
    private TunableDouble algaeD = new TunableDouble("Algae", "kD", Constants.AlgaeArm.kD);

    public AlgaeArmSubsystem(RobotContainer container, AlgaeArmIO io) {
        super(container, io, new AlgaeInputs(), new AlgaeOutputs());
    }

    @Override
    protected void processInputs() {}

    @Override
    protected void periodic(AlgaeInputs inputs, AlgaeOutputs outputs) {
        _stateManger.updateAlgaeArm(_inputs.angleRads);
        _inputs.pose = _stateManger.getPose(RobotCoordinate.ALGAE_ARM_BASE);

        if (algaeP.hasChanged() || algaeI.hasChanged() || algaeD.hasChanged()) {
            _io.setPID(algaeP.get(), algaeI.get(), algaeD.get(), 0.0, 0.0, 0.0, 0.0);
        }
    }

    public void setArmAngle(AlgaeState state) {
        setArmAngle(state.getArmAngle());
    }

    public void setArmAngle(double angle) {
        _outputs.desiredAngleRad = angle;
    }

    public double getArmAngleRads() {
        return _inputs.angleRads;
    }

    public void setAlgaeMotorVoltage(double voltage) {
        _outputs.voltageCommand = voltage;
    }

    public void setWheelMotorVoltage(double voltage) {
        _outputs.wheelVoltageCommand = voltage;
    }

    public boolean isAtDesiredAngle() {
        return Math.abs(_outputs.desiredAngleRad - _inputs.angleRads) < 0.04;
    }

    public boolean isAtState(AlgaeState state) {
        double angleDiff = Math.abs(state.getArmAngle() - getArmAngleRads());
        boolean isWithinPositionTolerance = angleDiff < Units.degreesToRadians(5.0);
        return isWithinPositionTolerance;
    }

    public boolean isSafeToEject() {
        return _inputs.angleRads > Constants.AlgaeArm.BOTTOM_EJECT_SAFE_ANGLE
                || _inputs.angleRads < Constants.AlgaeArm.TOP_EJECT_SAFE_ANGLE;
    }

    public boolean isAlgaeDetected() {
        return _inputs.isAlgaeDetected;
    }
}
