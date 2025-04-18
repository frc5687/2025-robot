package org.frc5687.robot.commands.drive;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;
import org.frc5687.robot.subsystems.superstructure.SuperstructureManager;
import org.frc5687.robot.util.FieldConstants;

public class DriveToProcessorLineup extends DriveWithNormalVectorAlignment {
    public DriveToProcessorLineup(DriveSubsystem drive, SuperstructureManager manager) {
        super(
                drive,
                manager,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                        return FlippingUtil.flipFieldPose(FieldConstants.Processor.processorLineUpClose);
                    } else {
                        return FieldConstants.Processor.processorLineUpClose;
                    }
                },
                true);
    }
}
