package org.frc5687.robot.subsystems.lights;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.subsystems.OutliersSubsystem;
import org.frc5687.robot.subsystems.SubsystemIO;

public class LightSubsystem extends OutliersSubsystem<LightInputs, LightOutputs> {
    private final RobotContainer _container;

    public LightSubsystem(RobotContainer container, SubsystemIO<LightInputs, LightOutputs> io) {
        super(container, io, new LightInputs(), new LightOutputs());
        _container = container;
    }

    @Override
    protected void processInputs() {
        RobotStateManager.getInstance();
    }

    @Override
    protected void periodic(LightInputs inputs, LightOutputs outputs) {
        if (DriverStation.isDisabled()) {
            Rotation2d heading = _container.getDrive().getHeading();
            Rotation2d targetHeading = Rotation2d.fromDegrees(120);

            Optional<Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                targetHeading = Rotation2d.fromDegrees(-60);
            }

            double errorDeg = targetHeading.minus(heading).getDegrees();
            if (errorDeg < -2.5) {
                outputs.desiredState = LightState.SOLID_GREEN;
            } else if (errorDeg > 2.5) {
                outputs.desiredState = LightState.PURPLE;
            } else { // just right
                if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
                    outputs.desiredState = LightState.BLUE;
                } else if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                    outputs.desiredState = LightState.RED;
                } else {
                    outputs.desiredState = LightState.SOLID_WHITE;
                }
            }

            return;
        }

        if (_container.getClimber().isSensorTriggered()) {
            outputs.desiredState = LightState.BLUE;
        } else if (_container.getSuperstructureManager().isAlgaeMode()) {
            if (_container.getAlgae().isAlgaeDetected()) {
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
}
