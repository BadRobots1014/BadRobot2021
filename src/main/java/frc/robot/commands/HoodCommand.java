/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;


public class HoodCommand extends CommandBase {
  private ShooterSubsystem m_shootSubsystem;
  private boolean m_isFinished = false;
  /**
   * Creates a new HoodCommand.
   */
  public HoodCommand(ShooterSubsystem subsystem) {
    m_shootSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    m_shootSubsystem.extendHood();
    m_isFinished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shootSubsystem.retractHood();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    if(m_isFinished){
      m_isFinished = false;
      return true;
    }
    else{
      return false;
    }
  }
}
