package org.frc5687.robot.subsystems.drive;

import org.frc5687.robot.util.BaseOutputs;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

@Logged
public class DriveOutputs extends BaseOutputs {
    @Logged(name = "Desired Chassis Speeds", importance = Logged.Importance.CRITICAL)
    public ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
    @Logged(name = "Modules/Desired States", importance = Logged.Importance.CRITICAL)
    public SwerveModuleState[] desiredStates = new SwerveModuleState[4];
}
