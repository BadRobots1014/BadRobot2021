package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.util.GyroProvider;

/**
 * A command that will turn the robot to the specified angle.
 */
public class AimCommand extends CommandBase {
  private final DriveTrainSubsystem m_driveTrain;
  private final GyroProvider m_gyro;

  private double m_desiredAngleToHold = 0;
  private DoubleSupplier m_centerXDoubleSupplier;
  
  private final PIDController m_pidController = new PIDController(0.028, 0, 0);
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive              The drive subsystem to use
   */

  public AimCommand(DriveTrainSubsystem driveTrain, GyroProvider gyro, DoubleSupplier centerXSupplier) {   
    
    addRequirements(driveTrain);

    m_gyro = gyro;
    m_driveTrain = driveTrain;
    m_centerXDoubleSupplier = centerXSupplier;
    // Set the controller to be continuous (because it is an angle controller)
    m_pidController.enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is
    // stationary at the
    // setpoint before it is considered as having reached the reference
    m_pidController.setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);

  }

  @Override
  public void initialize()
  {
    m_desiredAngleToHold = m_gyro.getHeading() + ((-5.0 / 11.0) * (m_centerXDoubleSupplier.getAsDouble() - 65));
    m_pidController.setSetpoint(m_desiredAngleToHold);
    System.out.println("Initialized holdcommand to hold angle at " + m_desiredAngleToHold);
  }

  @Override
  public void execute() 
  {
    m_desiredAngleToHold = m_gyro.getHeading() + ((-5.0 / 11.0) * (m_centerXDoubleSupplier.getAsDouble() - 65));
    m_pidController.setSetpoint(m_desiredAngleToHold);
    // m_desiredAngleToHold = m_angleDoubleSupplier.getAsDouble();
    double currentHeading = m_gyro.getHeading();

    System.out.println(currentHeading + " " + m_desiredAngleToHold);

    double pidOutput = m_pidController.calculate(currentHeading);
    // Clamp the output so it doesn't try to turn too fast
    double pidOutputClamped = MathUtil.clamp(pidOutput, -0.18, 0.18);
    double speedToTurnToCorrect = 0;
    if (Math.abs(pidOutputClamped) > 0.02) {
      speedToTurnToCorrect = pidOutputClamped * DriveConstants.kMaxAngularSpeed;
      m_driveTrain.arcadeDrive(0, speedToTurnToCorrect); 
    } else {
      m_driveTrain.arcadeDrive(0, 0);
    }

    SmartDashboard.putNumber("AutoAim SetPoint", m_pidController.getSetpoint());
    SmartDashboard.putNumber("Current Angle", currentHeading);
    SmartDashboard.putNumber("PID Output", pidOutputClamped);
    SmartDashboard.putNumber("Turn rate to correct (in radians)", speedToTurnToCorrect);
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}