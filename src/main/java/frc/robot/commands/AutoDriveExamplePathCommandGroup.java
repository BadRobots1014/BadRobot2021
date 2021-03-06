/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.util.RamseteUtil;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoDriveExamplePathCommandGroup extends SequentialCommandGroup {
  /**
   * Creates a new AutoDriveCommandGroup.
   */
  
  ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();

  public AutoDriveExamplePathCommandGroup(DriveTrainSubsystem driveTrain) { 
    
    
    // waypoints.add(new Pose2d(0, 0, new Rotation2d()));
    // waypoints.add(new Pose2d(1, 0, new Rotation2d()));

    // TrajectoryConfig config = new TrajectoryConfig(1, 1);

    // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, config);
    
    // addCommands(new RamseteCommand(trajectory, driveTrain::getPose, new RamseteController(), driveTrain.getDriveKinematics(), driveTrain::setSpeeds, driveTrain)
    // .beforeStarting(() -> driveTrain.setPose(new Pose2d(0, 0, new Rotation2d())), driveTrain)
    // .andThen(() -> System.out.println("Example Path Finished")));

    addCommands(RamseteUtil.getRamseteCommandForPath("paths/StraightLine.wpilib.json", driveTrain)
    .beforeStarting(() -> driveTrain.setPose(new Pose2d(0, 8.21055, new Rotation2d())))
    .andThen(() -> System.out.println("Example Path Finished")));

  }

  // @Override
  // public void initialize() {
  //   boolean autoColorRed = true;
  //   if (autoColorRed) {
  //     m_lights.setLightsPattern(LEDState.kRED);
  //   }
  //   else {
  //     m_lights.setLightsPattern(LEDState.kBLUE);
  //   }
  // }

  // For reference:
  // Create a voltage constraint to ensure we don't accelerate too fast
    // var autoVoltageConstraint =
    //     new DifferentialDriveVoltageConstraint(
    //         new SimpleMotorFeedforward(DriveConstants.ksVolts,
    //                                    DriveConstants.kvVoltSecondsPerMeter,
    //                                    DriveConstants.kaVoltSecondsSquaredPerMeter),
    //         m_driveTrain.getDriveKinematics(),
    //         10);

    //  // Create config for trajectory
    //  TrajectoryConfig config =
    //  new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
    //                       AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //      // Add kinematics to ensure max speed is actually obeyed
    //      .setKinematics(m_driveTrain.getDriveKinematics())
    //      // Apply the voltage constraint
    //      .addConstraint(autoVoltageConstraint);    
    
    // Trajectory firstTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(
    //         new Translation2d(1, 1),
    //         new Translation2d(2, -1)
    //     ),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     // Pass config
    //     config
    // );
}
