package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.util.GyroProvider;

/**
 * A command that will turn the robot to the specified angle.
 */
public class HoldPlaceCommand extends CommandBase {
  private final DriveTrainSubsystem m_driveTrain;
  private final GyroProvider m_gyro;

  private double m_desiredAngleToHold = 0;
  
  private final PIDController m_pidController = new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD);
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive              The drive subsystem to use
   */

  public HoldPlaceCommand(DriveTrainSubsystem driveTrain, GyroProvider gyro) {   
    
    addRequirements(driveTrain);

    m_gyro = gyro;
    m_driveTrain = driveTrain;
    // Set the controller to be continuous (because it is an angle controller)
    m_pidController.enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is
    // stationary at the
    // setpoint before it is considered as having reached the reference
    m_pidController.setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);

  }

  @Override
  public void initialize() {
    m_desiredAngleToHold = m_gyro.getHeading();
    m_pidController.setSetpoint(m_desiredAngleToHold);
  }

  @Override
  public void execute() {
    double currentHeading = m_gyro.getHeading();
    double pidOutput = m_pidController.calculate(currentHeading);
    // Clamp the output so it doesn't try to turn too fast
    double pidOutputClamped = MathUtil.clamp(pidOutput, -1, 1);
    double speedToTurnToCorrect = 0;
    

    if (Math.abs(pidOutputClamped) > 0.02) {
      speedToTurnToCorrect = pidOutputClamped * DriveConstants.kMaxAngularSpeed;
      m_driveTrain.arcadeDrive(0, speedToTurnToCorrect); 
    } else {
      m_driveTrain.arcadeDrive(0, 0);
    }
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}