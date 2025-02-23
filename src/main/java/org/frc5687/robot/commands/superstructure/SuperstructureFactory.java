package org.frc5687.robot.commands.superstructure;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.function.Supplier;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.commands.algae.AlgaeSetState;
import org.frc5687.robot.commands.coral.CoralSetState;
import org.frc5687.robot.commands.elevator.ElevatorSetState;
import org.frc5687.robot.subsystems.coralarm.CoralState;
import org.frc5687.robot.subsystems.superstructure.SuperstructureGoals;
import org.frc5687.robot.subsystems.superstructure.SuperstructureState;
import org.frc5687.robot.util.FieldConstants;

public class SuperstructureFactory {

    private static Command withStateTracking(
            RobotContainer container, SuperstructureState state, Command command) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> container.getSuperstructureTracker().setDesiredState(state)),
                command);
    }

    private static Command setSuperstructure(RobotContainer container, SuperstructureState state) {
        double prevHeight = container.getElevator().getInputs().heightPositionMeters;
        double newHeight = state.getElevator().getHeight();

        if (prevHeight == newHeight) {
            return new ParallelCommandGroup(
                    new ElevatorSetState(container.getElevator(), state.getElevator()),
                    new CoralSetState(container.getCoral(), state.getCoral()),
                    new AlgaeSetState(container.getAlgae(), state.getAlgae(), true)
                    /*new IntakeSetState(container.getIntake(), state.getIntake())*/ ); // FIXME
        } else {
            Command step1 =
                    new CoralSetState(
                            container.getCoral(), CoralState.IDLE_WITH_CORAL); // TODO algae fix this with algae
            Command step2 =
                    new ParallelCommandGroup(
                            new ElevatorSetState(container.getElevator(), state.getElevator()),
                            new AlgaeSetState(container.getAlgae(), state.getAlgae(), true)
                            /*new IntakeSetState(container.getIntake(), state.getIntake())*/ ); // FIXME
            Command step3 = new CoralSetState(container.getCoral(), state.getCoral());
            return new SequentialCommandGroup(step1, step2, step3);
        }
    }

    public static Command clearIntake(RobotContainer container) {
        return setSuperstructure(container, SuperstructureGoals.CLEAR_INTAKE)
                .unless(() -> !container.getSuperstructureTracker().needToClearIntake());
    }

    public static Command transitionToSafeCoralState(RobotContainer container) {
        return setSuperstructure(container, SuperstructureGoals.SAFE_CORAL_TRANSITION)
                .unless(() -> !container.getSuperstructureTracker().needsSafeCoralTransition());
    }

    public static Command ensureClearance(RobotContainer container) {
        return new SequentialCommandGroup(
                clearIntake(container), transitionToSafeCoralState(container));
    }

    public static Command receiveFromFunnel(RobotContainer container) {
        return withStateTracking(
                container,
                SuperstructureGoals.RECEIVE_FROM_FUNNEL,
                new SequentialCommandGroup(
                        ensureClearance(container),
                        new SuperstructureReceive(container, SuperstructureGoals.RECEIVE_FROM_FUNNEL)));
    }

    // This bypasses coral detection
    public static Command receiveFromFunnelSim(RobotContainer container) {
        return withStateTracking(
                container,
                SuperstructureGoals.RECEIVE_FROM_FUNNEL,
                new SequentialCommandGroup(
                        ensureClearance(container),
                        setSuperstructure(container, SuperstructureGoals.RECEIVE_FROM_FUNNEL)));
    }

    public static Command placeCoralL4(
            RobotContainer container, boolean withAlgaeGrab, Supplier<Boolean> overrideButton) {
        var targetState =
                withAlgaeGrab
                        ? SuperstructureGoals.PLACE_CORAL_L4_ALGAE_GRAB
                        : SuperstructureGoals.PLACE_CORAL_L4;
        return nameThisBetter(container, targetState, overrideButton);
    }

    public static Command placeCoralL3(
            RobotContainer container, boolean withAlgaeGrab, Supplier<Boolean> overrideButton) {
        var targetState =
                withAlgaeGrab
                        ? SuperstructureGoals.PLACE_CORAL_L3_ALGAE_GRAB
                        : SuperstructureGoals.PLACE_CORAL_L3;
        return nameThisBetter(container, targetState, overrideButton);
    }

    public static Command placeCoralL2(RobotContainer container, Supplier<Boolean> overrideButton) {
        return nameThisBetter(container, SuperstructureGoals.PLACE_CORAL_L2, overrideButton);
    }

    public static Command placeCoralL1(RobotContainer container, Supplier<Boolean> overrideButton) {
        return nameThisBetter(container, SuperstructureGoals.PLACE_CORAL_L1, overrideButton);
    }

    public static Command nameThisBetter(
            RobotContainer container, SuperstructureState targetState, Supplier<Boolean> overrideButton) {
        double targetElevatorMotorMeters =
                targetState.getElevator().getHeight() / 2.0; // FIXME is this right??

        return withStateTracking(
                container,
                targetState,
                new SequentialCommandGroup(
                        new InstantCommand(
                                () -> {
                                    System.out.println("🚨🚨🚨🚨🚨🚨🚨🚨");
                                }),
                        ensureClearance(container),
                        new QueueElevatorCommand(
                                () -> {
                                    Translation2d currentPose = container.getDrive().getPose().getTranslation();
                                    var alliance = DriverStation.getAlliance();
                                    if (alliance.isEmpty()) {
                                        System.err.println("Alliance was Empty in QueueElevatorCommand");
                                        return 0.0; // default to at least moving the mechanism...
                                    }
                                    Translation2d reefCenter = FieldConstants.Reef.center;
                                    if (alliance.get() == Alliance.Red) {
                                        reefCenter = FlippingUtil.flipFieldPosition(reefCenter);
                                    }
                                    return currentPose.getDistance(reefCenter) - 1.5;
                                },
                                () -> {
                                    return container
                                            .getElevator()
                                            .getSignedTimeToSetpoint(2.0, targetElevatorMotorMeters);
                                },
                                overrideButton,
                                setSuperstructure(container, targetState))));
    }

    public static Command grabAlgaeL1(RobotContainer container) {
        return withStateTracking(
                container,
                SuperstructureGoals.L1_ALGAE_GRAB,
                new SequentialCommandGroup(
                        ensureClearance(container),
                        setSuperstructure(container, SuperstructureGoals.L1_ALGAE_GRAB),
                        new WaitUntilAlgaeDetected(container)));
    }

    public static Command grabAlgaeL2(RobotContainer container) {
        return withStateTracking(
                container,
                SuperstructureGoals.L2_ALGAE_GRAB,
                new SequentialCommandGroup(
                        ensureClearance(container),
                        setSuperstructure(container, SuperstructureGoals.L2_ALGAE_GRAB),
                        new WaitUntilAlgaeDetected(container)));
    }

    public static Command processorDropoff(RobotContainer container) {
        return withStateTracking(
                        container,
                        SuperstructureGoals.PROCESSOR_DROPOFF,
                        setSuperstructure(container, SuperstructureGoals.PROCESSOR_DROPOFF))
                .andThen(
                        withStateTracking(
                                container,
                                SuperstructureGoals.PROCESSOR_DROPOFF_WHEEL,
                                setSuperstructure(container, SuperstructureGoals.PROCESSOR_DROPOFF_WHEEL)));
    }
}
