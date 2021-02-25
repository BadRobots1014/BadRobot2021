/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GathererSubsystem;

/**
 * Retracts Gatherer when it is run
 */
public class RetractGathererCommand extends CommandBase {
  private final GathererSubsystem m_gatherer;

  public RetractGathererCommand(GathererSubsystem gatherer) {
    m_gatherer = gatherer;
    
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
