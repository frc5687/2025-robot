package org.frc5687.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.frc5687.robot.RobotContainer;

public class AutoActions {

    public static Command autoPlace(RobotContainer container) {
        return new SequentialCommandGroup(
                new InstantCommand(
                        () -> {
                            container.getCoral().setWheelMotorDutyCycle(1);
                        }),
                new WaitCommand(0.3),
                new InstantCommand(
                        () -> {
                            container.getCoral().setWheelMotorDutyCycle(0);
                        })
                );
    }
}
