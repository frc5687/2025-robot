package org.frc5687.robot.subsystems.drive;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.frc5687.robot.util.BaseOutputs;

@Logged
public class DriveOutputs extends BaseOutputs {
    @Logged(name = "Modules/Desired States", importance = Logged.Importance.DEBUG)
    public SwerveModuleState[] desiredStates = new SwerveModuleState[4];
}
