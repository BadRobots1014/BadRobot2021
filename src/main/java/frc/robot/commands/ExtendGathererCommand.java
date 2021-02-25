/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GathererSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ExtendGathererCommand extends CommandBase {
  private final GathererSubsystem m_gatherer;
  /**
   * Creates a new GathererOutCommand.
   */
  public ExtendGathererCommand(GathererSubsystem gatherer) {
    m_gatherer = gatherer;
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    // super(
    //   new InstantCommand(() -> gatherer.gathererOut(), gatherer),
    //   new RunCommand(() -> gatherer.runGatherer(), gatherer)
    // );
    addRequirements(gatherer);
  }

  @Override
  public void initialize() {
    m_gatherer.gathererOut();
    m_gatherer.runGatherer();
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {
    m_gatherer.stopGather();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}


