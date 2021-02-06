/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GathererSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDState;
import frc.robot.util.RamseteUtil;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoDriveForewardCommand extends SequentialCommandGroup {
  /**
   * Creates a new AutoDriveCommandGroup.
   */
  LEDSubsystem m_lights;
  public AutoDriveForewardCommand(DriveTrainSubsystem m_drive, ShooterSubsystem m_shooter, MagazineSubsystem m_magazine, GathererSubsystem m_gatherer) {    
    //m_lights = lights;
    // Before starting, set the pose to 0, -3, because that's where the path starts in the Example that was created.
    addCommands(new RunCommand(() -> m_drive.arcadeDrive(-1.25, 0), m_drive).withTimeout(1.0).andThen(() -> m_drive.stop()));
  }
}
