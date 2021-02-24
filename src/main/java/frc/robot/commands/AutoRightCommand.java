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
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GathererSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.RamseteUtil;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoRightCommand extends SequentialCommandGroup {
    /**
     * Creates a new AutoDriveCommandGroup.
     */
    LEDSubsystem m_lights;

    public AutoRightCommand(DriveTrainSubsystem driveTrain, ShooterSubsystem shooter, 
    GathererSubsystem gatherer, MagazineSubsystem magazine) {
    //m_lights = lights;
    // Before starting, set the pose to 0, -3, because that's where the path starts in the Example that was created.
    addCommands(RamseteUtil.getRamseteCommandForPath("paths/RedRightStart.wpilib.json", driveTrain)
                .beforeStarting(() -> driveTrain.setPose(new Pose2d(2.995, -5.855, new Rotation2d(0))))
                .andThen(() -> driveTrain.stop()),
                new ShootContinuousForTimeCommand(gatherer, magazine, shooter, ShooterConstants.kShootThreeBallsTime)
                .andThen(() -> driveTrain.stop()), 
                RamseteUtil.getRamseteCommandForPath("paths/RedToLeft.wpilib.json", driveTrain)
                .andThen(() -> driveTrain.stop()),
                RamseteUtil.getRamseteCommandForPath("paths/RedLeftCollect.wpilib.json", driveTrain)
                .raceWith(new GatherCommand(gatherer))
                .andThen(() -> driveTrain.stop())
                .andThen(() -> gatherer.stopGather())
    );
  }
}

 