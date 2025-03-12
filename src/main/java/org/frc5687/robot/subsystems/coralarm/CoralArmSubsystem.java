package org.frc5687.robot.subsystems.coralarm;

import edu.wpi.first.math.util.Units;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.RobotStateManager.RobotCoordinate;
import org.frc5687.robot.subsystems.OutliersSubsystem;
import org.frc5687.robot.util.TunableDouble;

public class CoralArmSubsystem extends OutliersSubsystem<CoralInputs, CoralOutputs> {

    private final RobotStateManager _stateManger = RobotStateManager.getInstance();

    private TunableDouble coralP = new TunableDouble("Coral", "kP", Constants.CoralArm.kP);
    private TunableDouble coralI = new TunableDouble("Coral", "kI", Constants.CoralArm.kI);
    private TunableDouble coralD = new TunableDouble("Coral", "kD", Constants.CoralArm.kD);
    private TunableDouble coralV = new TunableDouble("Coral", "kV", Constants.CoralArm.kV);
    private TunableDouble coralS = new TunableDouble("Coral", "kS", Constants.CoralArm.kS);
    private TunableDouble fudge = new TunableDouble("Coral", "fudge", 0.0);

    public CoralArmSubsystem(RobotContainer container, CoralArmIO io) {
        super(container, io, new CoralInputs(), new CoralOutputs());
    }

    @Override
    protected void processInputs() {}

    @Override
    protected void periodic(CoralInputs inputs, CoralOutputs outputs) {
        _stateManger.updateCoralArm(_inputs.angleRads);
        _inputs.pose = _stateManger.getPose(RobotCoordinate.CORAL_ARM_BASE);
        double elevatorAcceleration = _robotContainer.getElevator().getInputs().platformAcceleration;
        _outputs.dynamicsFF =
                calculateGravityFeedForward(getArmAngleRads(), 9.81 + fudge.get() * elevatorAcceleration);

        if (coralP.hasChanged()
                || coralI.hasChanged()
                || coralD.hasChanged()
                || coralV.hasChanged()
                || coralS.hasChanged()) {
            _io.setPID(coralP.get(), coralI.get(), coralD.get(), coralV.get(), coralS.get(), 0.0, 0.0);
        }
        var command = getCurrentCommand();
        log("Active Command", command == null ? "null" : command.getName());
    }

    private double calculateGravityFeedForward(double angle, double gravityAcceleration) {
        return ((Constants.CoralArm.ARM_LENGTH / 2.0)
                        * (Constants.CoralArm.GEARBOX.rOhms * Constants.CoralArm.ARM_MASS * gravityAcceleration)
                        / (Constants.CoralArm.GEAR_RATIO * Constants.CoralArm.GEARBOX.KtNMPerAmp))
                * Math.cos(angle);
    }

    public void setWheelMotorDutyCycle(double dutyCycle) {
        _outputs.wheelPositionControl = false;
        _outputs.wheelDutyCycle = dutyCycle;
    }

    public void setWheelMotorPosition(double position) {
        _outputs.wheelPositionControl = true;
        _outputs.wheelDutyCycle = 0;
        _outputs.wheelPositionCommand = position;
    }

    public void holdPosition() {
        setWheelMotorPosition(getWheelMotorPosition());
    }

    public void setArmAngle(CoralState state) {
        setArmAngle(state.getArmAngle());
    }

    public double getArmAngleRads() {
        return _inputs.angleRads;
    }

    public double getWheelMotorPosition() {
        return _inputs.wheelAngle;
    }

    public void setArmAngle(double angleRads) {
        _outputs.desiredAngleRad = angleRads;
    }

    public boolean isAtDesiredAngle() {
        return Math.abs(_outputs.desiredAngleRad - _inputs.angleRads) < 0.08;
    }

    public boolean isAtState(CoralState state) {
        double angleDiff = Math.abs(state.getArmAngle() - getArmAngleRads());
        boolean isWithinPositionTolerance = angleDiff < Units.degreesToRadians(3.0);
        return isWithinPositionTolerance;
    }

    public boolean isAtDesiredWheelAngle() {
        return Math.abs(_outputs.wheelPositionCommand - _inputs.wheelAngle) < 0.08;
    }

    public boolean isCoralDetected() {
        return _inputs.isCoralDetected;
    }
}
