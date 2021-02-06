/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.GathererSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class GathererInCommand extends CommandBase {
  private final GathererSubsystem m_gatherer;
  /**
   * Creates a new GathererInCommand.
   */
  public GathererInCommand(GathererSubsystem gatherer) {
    m_gatherer = gatherer;
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    
    // super(
    //   new InstantCommand(() -> gatherer.gathererIn(), gatherer),
    //   new InstantCommand(() -> gatherer.stopGather(), gatherer)
    // );
    
    addRequirements(gatherer);
  }

  @Override
  public void initialize() {
    m_gatherer.gathererIn();
    m_gatherer.stopGather();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
