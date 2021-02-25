/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.GathererSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SingleFireCommandGroup extends ParallelDeadlineGroup {
  /**
   * Creates a new SingleFireCommandGroup.
   */

  /*
  The plan if this doesn't work:
  DO IT MANUALLY.
  1. EXTEND THE GATHERER!--PUSH RIGHT BUMPER!
  2. Rev up the shooter--use the right trigger and joystick, or create a button binding for RunShooterCommand
  3. Use the button controls for the magazine--PUSH Y!
  4. REMEMBER TO DO 2 AND 3 SIMULTANEOUSLY! YOU NEED TO KEEP THE SHOOTER RUNNING!
  */
  public SingleFireCommandGroup(ShooterSubsystem shooterSubsystem, MagazineSubsystem magSubsystem, GathererSubsystem gathererSubsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new SequentialCommandGroup( // Deadline command
        new WaitUntilCommand(() -> {
          if (shooterSubsystem.getDeltaDesiredVelocity() >= -ShooterConstants.kFeedThresholdAngularSpeedDelta
          && shooterSubsystem.getDeltaDesiredVelocity() <= ShooterConstants.kFeedThresholdAngularSpeedDelta) {
            return true;
          } else {
            return false;
          }
        }),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(() -> { return shooterSubsystem.getDeltaDesiredVelocity() <= ShooterConstants.kShootThresholdAngularSpeedDelta; }),//.withTimeout(6.0)
          new RunMagazineCommand(magSubsystem).withTimeout(1.0)
          // new SequentialCommandGroup(
          //   new WaitCommand(2.0),)
          //   new ParallelRaceGroup(
          //     new RunGathererReversedCommand(gathererSubsystem),
          //     new WaitCommand(1.0)
          //   ),
          //   new GatherCommand(gathererSubsystem)
          // ),
        )//.withTimeout(3.0)//,
        // new WaitCommand(0.5) // Do we need this or can it do without?
      ),
      new SequentialCommandGroup( // Make sequential: cannot use a subsystem in two places at once
        new ExtendGathererCommand(gathererSubsystem),
        new RunCommand(gathererSubsystem::stopGather, gathererSubsystem) // IF this doesn't override the default command...
      ),
      new RunCommand(shooterSubsystem::runShooter, shooterSubsystem)
    );
  }
}
