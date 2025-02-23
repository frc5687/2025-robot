package org.frc5687.robot.subsystems.superstructure;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import java.util.function.Supplier;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.subsystems.intake.IntakeState;
import org.frc5687.robot.util.FieldConstants;
import org.frc5687.robot.util.TunableDouble;

public class SuperstructureManager extends SubsystemBase {
    private final RobotContainer _container;
    private final RequestHandler _requestHandler;
    private boolean _forceQueueExecution = false;

    public SuperstructureManager(RobotContainer container) {
        _container = container;
        _requestHandler = new RequestHandler(container);
    }

    /**
     * This crates a request to the superstructure for a desired state and motor values while having
     * the option to queue
     *
     * @param stateSupplier A supplier for the desired superstructure state positions (elevator
     *     height, arm angles, etc)
     * @param motors The motor values for certian actions (coral eject, algae hold etc)
     * @param description debugging description of the action
     * @param type RequestType to queue or not to queue (that is the question is it not)?
     * @return
     */
    public Command createRequest(
            Supplier<SuperstructureState> stateSupplier,
            MotorState motors,
            String description,
            RequestType type) {
        return new FunctionalCommand(
                // init
                () ->
                        _requestHandler.handleNewRequest(
                                new SuperstructureRequest(
                                        stateSupplier.get(),
                                        motors,
                                        type,
                                        () ->
                                                type != RequestType.QUEUED
                                                        || _forceQueueExecution
                                                        || isPositionSafe()
                                                        || isElevatorGoingDown(stateSupplier.get()),
                                        description)),
                // execute
                () -> {},
                // end
                (interrupted) -> {
                    if (interrupted) {
                        System.out.println("Request interrupted: " + description);
                    }
                },
                // IsFinished
                () -> {
                    SuperstructureRequest activeRequest = _requestHandler.getActiveRequest();
                    return activeRequest == null || !activeRequest.description().equals(description);
                },
                this);
    }

    public Command setToPlaceHeight(SuperstructureState placeState, RequestType type) {
        return createRequest(
                () -> placeState, MotorState.HOLD, "Set place height " + placeState.getElevator(), type);
    }

    public Command placeAtCurrentHeight(RequestType type) {
        return new SequentialCommandGroup(
                createRequest(this::getCurrentGoal, MotorState.EJECT, "Eject at current height", type),
                new WaitCommand(0.5),
                createRequest(
                        () -> SuperstructureGoals.RECEIVE_FROM_FUNNEL,
                        MotorState.STOPPED,
                        "Return to funnel",
                        RequestType.IMMEDIATE));
    }

    public Command groundIntake(RequestType type) {
        return createRequest(
                () -> SuperstructureGoals.GROUND_PICKUP, MotorState.INTAKE, "Ground Intake", type);
    }

    // public Command receiveFunnel(RequestType type) {
    //     return createRequest(
    //                     () -> SuperstructureGoals.RECEIVE_FROM_FUNNEL,
    //                     MotorState.RECEIVE_FUNNEL,
    //                     "Receive from funnel",
    //                     type)
    //             .until(() -> _container.getCoral().isCoralDetected())
    //             .andThen(
    //                     createRequest(
    //                             () -> SuperstructureGoals.RECEIVE_FROM_FUNNEL,
    //                             MotorState.HOLD,
    //                             "Hold after receive",
    //                             RequestType.IMMEDIATE));
    // }

    public Command receiveFunnel(RequestType type) {
        return new SequentialCommandGroup(
                createRequest(
                        () -> SuperstructureGoals.SAFE_CORAL_TRANSITION,
                        MotorState.HOLD,
                        "Moving to safe state for receive",
                        RequestType.IMMEDIATE),
                createRequest(
                                () -> SuperstructureGoals.RECEIVE_FROM_FUNNEL,
                                MotorState.RECEIVE_FUNNEL,
                                "Receiving from funnel",
                                type)
                        .until(() -> _container.getCoral().isCoralDetected()),
                new FunctionalCommand(
                        () -> {
                            double currentPos = _container.getCoral().getWheelMotorPosition();
                            double goMore = new TunableDouble("Receive", "goMore", 1.5).get();
                            _container.getCoral().setWheelMotorPosition(currentPos + goMore);
                        },
                        () -> {},
                        (interrupted) -> {},
                        () -> _container.getCoral().isAtDesiredAngle(),
                        _container.getCoral()),
                createRequest(
                        () -> SuperstructureGoals.RECEIVE_FROM_FUNNEL,
                        MotorState.HOLD,
                        "Holding after receive",
                        RequestType.IMMEDIATE));
    }

    public Command receiveFunnelSim(RequestType type) {
        return createRequest(
                        () -> SuperstructureGoals.RECEIVE_FROM_FUNNEL,
                        MotorState.RECEIVE_FUNNEL,
                        "Receive from funnel",
                        type)
                .until(() -> true)
                .andThen(
                        createRequest(
                                () -> SuperstructureGoals.RECEIVE_FROM_FUNNEL,
                                MotorState.HOLD,
                                "Hold after receive",
                                RequestType.IMMEDIATE));
    }

    public Command grabAlgae(SuperstructureState state, RequestType type) {
        return createRequest(
                        () -> state, MotorState.ALGAE_INTAKE, "Grab algae at " + state.getElevator(), type)
                .until(() -> _container.getAlgae().isAlgaeDetected());
    }

    // Queue override controls
    public void forceQueueExecution() {
        _forceQueueExecution = true;
    }

    public void releaseQueueExecution() {
        _forceQueueExecution = false;
    }

    private boolean isPositionSafe() {
        Translation2d currentPose = _container.getDrive().getPose().getTranslation();
        Translation2d reefCenter = getAllianceSpecificReefCenter();
        return currentPose.getDistance(reefCenter) <= 3.5;
    }

    private boolean isElevatorGoingDown(SuperstructureState requestedState) {
        double elevatorHeight = _container.getElevator().getPlatformWorldHeight();
        return requestedState.getElevator().getHeight() < elevatorHeight;
    }

    private Translation2d getAllianceSpecificReefCenter() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return FieldConstants.Reef.center;
        return alliance.get() == DriverStation.Alliance.Red
                ? FlippingUtil.flipFieldPosition(FieldConstants.Reef.center)
                : FieldConstants.Reef.center;
    }

    @Override
    public void periodic() {
        _requestHandler.execute();
    }

    private SuperstructureState getCurrentGoal() {
        return new SuperstructureState(
                _container.getElevator().getDesiredState(),
                _container.getCoral().getDesiredState(),
                _container.getAlgae().getDesiredState(),
                IntakeState.IDLE); // TODO FIX INTAKE
    }
}
