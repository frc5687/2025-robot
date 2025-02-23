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
        setDesiredState(_inputs.algaeState);
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

    public AlgaeState getDesiredState() {
        return _outputs.desiredState;
    }

    public void setDesiredState(AlgaeState state) {
        _outputs.desiredState = state;
        setDesiredAngleRadians(state.getArmAngle());
    }

    public void setDesiredAngleRadians(double angleRadians) {
        _outputs.desiredAngleRad = angleRadians;
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

    // TODO: Make tolerance for all subsystems
    public boolean isAtDesiredAngle() {
        return Math.abs(_outputs.desiredAngleRad - _inputs.angleRads) < 0.04;
    }

    public void setCurrentState(AlgaeState state) {
        _inputs.algaeState = state;
    }

    public AlgaeState getCurrentState() {
        return _inputs.algaeState;
    }

    public boolean isAlgaeDetected() {
        return _inputs.isAlgaeDetected;
    }

    public void mapToClosestState() {
        AlgaeState closestState = AlgaeState.IDLE;
        double minDist = Double.MAX_VALUE;
        for (AlgaeState state : AlgaeState.values()) {
            double dist = Math.abs(getArmAngleRads() - state.getArmAngle());
            if (dist < minDist) {
                closestState = state;
                minDist = dist;
            }
        }
        _inputs.algaeState = closestState;
    }

    public boolean isAtState(AlgaeState state) {
        double angleDiff = Math.abs(state.getArmAngle() - getArmAngleRads());
        boolean isWithinPositionTolerance = angleDiff < Units.degreesToRadians(5.0);
        return isWithinPositionTolerance;
    }
}
