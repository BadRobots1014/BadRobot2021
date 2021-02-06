/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.MagazineConstants;
import frc.robot.subsystems.MagazineSubsystem;

public class IndexMagazineCommand extends CommandBase {
  private final MagazineSubsystem m_magSubsystem;

  private final ShuffleboardTab m_shooterTab = Shuffleboard.getTab("Shooting");

  private final NetworkTableEntry m_inSensorState = m_shooterTab.add("Magazine input sensor state", true).getEntry();

  private final Timer m_timer = new Timer();

  /**
   * Creates a new IndexMagazineCommand.
   */
  public IndexMagazineCommand(MagazineSubsystem magSubsystem) {
    m_magSubsystem = magSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(magSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_inSensorState.getBoolean(true)) {
      m_magSubsystem.runMotor();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_magSubsystem.stopMotor();
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer.get() > MagazineConstants.kDelay && m_inSensorState.getBoolean(true)) {
      return true;
    } else {
      return false;
    }
  }
}
