/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.util.GyroProvider;

public class TurnCommand extends CommandBase {
    private final DriveTrainSubsystem m_driveTrain;
    private final GyroProvider m_gyro;
    private final double m_angle;

    private final PIDController m_pidController = new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD);
  /**
   * Creates a new TurnCommand.
   * @param driveTrain The drive train to use
   * @param gyro The gyro to use
   * @param angle The angle to turn
   */
  public TurnCommand(DriveTrainSubsystem driveTrain, GyroProvider gyro, double angle) {
    m_driveTrain = driveTrain;
    m_gyro = gyro;
    m_angle = angle;

    m_pidController.enableContinuousInput(-180, 180);
    m_pidController.setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);

    addRequirements(driveTrain);
  }

  /**
   * Creates a new TurnCommand.
   * @param driveTrain The drive train to use
   * @param gyro The gyro to use
   * @param angle The angle to turn
   * @param angleTolerance The tolerance for the PID algorithm
   */
  public TurnCommand(DriveTrainSubsystem driveTrain, GyroProvider gyro, double angle, double angleTolerance) {
    m_driveTrain = driveTrain;
    m_gyro = gyro;
    m_angle = angle;

    m_pidController.enableContinuousInput(-180, 180);
    m_pidController.setTolerance(angleTolerance, DriveConstants.kTurnRateToleranceDegPerS);

    addRequirements(driveTrain);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double setPoint;
    if (m_gyro.getHeading() + m_angle < -180) {
      setPoint = m_gyro.getHeading() + m_angle + 360;
    } else if (m_gyro.getHeading() + m_angle > 180) {
      setPoint = m_gyro.getHeading() + m_angle - 360;
    } else {
      setPoint = (m_gyro.getHeading() + m_angle);
    }

    m_pidController.setSetpoint(setPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.arcadeDrive(0, m_pidController.calculate(m_gyro.getHeading()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {    
      m_driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pidController.atSetpoint();
  }
}