package org.frc5687.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.commands.coral.IntakeCoralWheels;
import org.frc5687.robot.subsystems.superstructure.SuperstructureGoals;

public class SuperstructureFactory {
    public static Command grabAlgaeL1(RobotContainer container) {
        return new SetSuperstructure(
                container, SuperstructureGoals.L1_ALGAE_GRAB) /*.andThen() spin the wheels*/;
    }

    public static Command grabAlgaeL2(RobotContainer container) {
        return new SetSuperstructure(
                container, SuperstructureGoals.L2_ALGAE_GRAB) /*.andThen() spin the wheels*/;
    }

    public static Command processorDropoff(RobotContainer container) {
        return new SetSuperstructure(
                container, SuperstructureGoals.PROCESSOR_DROPOFF) /*.andThen() spin the wheels*/;
    }

    public static Command receiveFromFunnel(RobotContainer container) {
        return new SetSuperstructure(container, SuperstructureGoals.RECEIVE_FROM_FUNNEL)
                .andThen(new IntakeCoralWheels(container));
    }
}
