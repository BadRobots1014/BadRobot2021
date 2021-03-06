/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.util.RamseteUtil;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoNavBarrelRacingCommand extends SequentialCommandGroup {
    /**
     * Creates a new AutoDriveCommandGroup.
     */
    LEDSubsystem m_lights;

    public AutoNavBarrelRacingCommand(DriveTrainSubsystem driveTrain) {
    //m_lights = lights;
    // Before starting, set the pose to 0, -3, because that's where the path starts in the Example that was created.
    addCommands( RamseteUtil.getRamseteCommandForPath("paths/BarrelRacing.wpilib.json", driveTrain)
                .beforeStarting(() -> driveTrain.setPose(new Pose2d(1.219893073040874, 2.0944329210803616, new Rotation2d(0.44024051355478006))))
                .andThen(() -> driveTrain.stop())
    );
  }
}

 