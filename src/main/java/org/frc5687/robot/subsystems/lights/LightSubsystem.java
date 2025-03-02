package org.frc5687.robot.subsystems.lights;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.subsystems.OutliersSubsystem;
import org.frc5687.robot.subsystems.SubsystemIO;
import org.frc5687.robot.util.FieldConstants;

public class LightSubsystem extends OutliersSubsystem<LightInputs, LightOutputs> {
    private final RobotContainer _container;
    private double prevBlink;

    private static final double DUTY_CYCLE = 0.5;
    private static final double BARGE_TARGET_X =
            FieldConstants.fieldLength / 2.0
                    - Units.inchesToMeters(64); // 64in from barge center 3/1/25 xavier

    public LightSubsystem(RobotContainer container, SubsystemIO<LightInputs, LightOutputs> io) {
        super(container, io, new LightInputs(), new LightOutputs());
        _container = container;
        prevBlink = 0.0;
    }

    @Override
    protected void processInputs() {
        RobotStateManager.getInstance();
    }

    @Override
    protected void periodic(LightInputs inputs, LightOutputs outputs) {
        if (_container.getClimber().isSensorTriggered()) {
            outputs.desiredState = LightState.BLUE;
        } else if (_container.getSuperstructureManager().isAlgaeMode()) {
            if (_container.getAlgae().isAlgaeDetected()) {
                double driveX = _container.getDrive().getPose().getX();
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                    driveX = FieldConstants.fieldLength - driveX;
                }
                double distance = Math.abs(BARGE_TARGET_X - driveX);

                if (distance < Units.inchesToMeters(4)) {
                    outputs.desiredState = LightState.FIRE;
                }
                outputs.desiredState = LightState.FLASHING_GREEN;
            } else {
                outputs.desiredState = LightState.SOLID_GREEN;
            }
        } else { // coral mode
            if (_container.getCoral().isCoralDetected()) {
                outputs.desiredState = LightState.FLASHING_WHITE;
            } else {
                outputs.desiredState = LightState.SOLID_WHITE;
            }
        }
    }

    private LightState blink(double periodSeconds, LightState color, LightState color2) {
        double timestamp = Timer.getFPGATimestamp();
        double diff = timestamp - prevBlink;
        if (diff > periodSeconds) {
            prevBlink = timestamp;
            return color;
        } else if (diff > periodSeconds * DUTY_CYCLE) {
            return color2;
        } else {
            return color;
        }
    }
}
