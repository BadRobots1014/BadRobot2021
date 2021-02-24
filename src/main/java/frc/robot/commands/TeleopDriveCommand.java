/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class TeleopDriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrainSubsystem m_driveTrain; 

  // Initialize these so that it is not empty.
  private DoubleSupplier m_leftDoubleSupplier = () -> 0.0;
  private DoubleSupplier m_rightDoubleSupplier = () -> 0.0;
  private BooleanSupplier m_leftSlowSupplier = () -> false;
  private BooleanSupplier m_rightSlowSupplier = () -> false;

  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(5);

  // Arcade Drive Stuff
  public static final double kDefaultDeadband = 0.02;
  public static final double kDefaultMaxOutput = 1.0;
  private static final boolean kSquareInputs = true; 

  // Curvature Drive Stuff
  public static final double kDefaultQuickStopThreshold = 0.2;
  public static final double kDefaultQuickStopAlpha = 0.1;
  private double m_quickStopThreshold = kDefaultQuickStopThreshold;
  private double m_quickStopAlpha = kDefaultQuickStopAlpha;
  private double m_quickStopAccumulator;

  /**
   * Creates a new TeleopDriveCommand.
   *
   * @param driveTrain The DriveTrainSubsystem used by this command.
   */
  public TeleopDriveCommand(DriveTrainSubsystem driveTrain) {
    m_driveTrain = driveTrain;

    addRequirements(driveTrain);
  }

  public void setControllerSupplier(DoubleSupplier leftDoubleSupplier, DoubleSupplier rightDoubleSupplier, BooleanSupplier leftSlowSupplier, BooleanSupplier rightSlowSupplier) {

    // TODO: There's probably a better way to write this
    m_leftDoubleSupplier = () -> {
      if (Math.abs(leftDoubleSupplier.getAsDouble()) < DriveConstants.kDeadBand) {
        return (double) 0;
      } else {
        return leftDoubleSupplier.getAsDouble();
      }
    };

    m_rightDoubleSupplier = () -> {
      if (Math.abs(rightDoubleSupplier.getAsDouble()) < DriveConstants.kDeadBand) {
        return (double) 0;
      } else {
        return rightDoubleSupplier.getAsDouble();
      }
    };

    m_leftSlowSupplier = leftSlowSupplier;
    m_rightSlowSupplier = rightSlowSupplier;
  } 

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double xSpeed = m_leftDoubleSupplier.getAsDouble();
    // double zRotation = m_rightDoubleSupplier.getAsDouble();
    // boolean quickTurn = m_quickTurnSupplier.getAsBoolean();
    
    // arcadeDrive(xSpeed, zRotation);
    //curvatureDrive(xSpeed, zRotation, quickTurn);
    if (m_leftSlowSupplier.getAsBoolean() && m_rightSlowSupplier.getAsBoolean()) { 
      m_driveTrain.tankDrive(DriveConstants.kSlowFactor*m_leftDoubleSupplier.getAsDouble(), DriveConstants.kSlowFactor*m_rightDoubleSupplier.getAsDouble());
    } else if (m_leftSlowSupplier.getAsBoolean()) {
      m_driveTrain.tankDrive(DriveConstants.kSlowFactor*m_leftDoubleSupplier.getAsDouble(), m_rightDoubleSupplier.getAsDouble());
    } else if (m_rightSlowSupplier.getAsBoolean()) {
      m_driveTrain.tankDrive(m_leftDoubleSupplier.getAsDouble(), DriveConstants.kSlowFactor*m_rightDoubleSupplier.getAsDouble());
    } else {
      m_driveTrain.tankDrive(m_leftDoubleSupplier.getAsDouble(), m_rightDoubleSupplier.getAsDouble());
    }
  }

  private void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {

    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    xSpeed = applyDeadband(xSpeed, kDefaultDeadband);

    zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
    zRotation = applyDeadband(zRotation, kDefaultDeadband);

    double angularPower;
    boolean overPower;

    if (isQuickTurn) {
      if (Math.abs(xSpeed) < m_quickStopThreshold) {
        m_quickStopAccumulator = (1 - m_quickStopAlpha) * m_quickStopAccumulator
            + m_quickStopAlpha * MathUtil.clamp(zRotation, -1.0, 1.0) * 2;
      }
      overPower = true;
      angularPower = zRotation;
    } else {
      overPower = false;
      angularPower = Math.abs(xSpeed) * zRotation - m_quickStopAccumulator;

      if (m_quickStopAccumulator > 1) {
        m_quickStopAccumulator -= 1;
      } else if (m_quickStopAccumulator < -1) {
        m_quickStopAccumulator += 1;
      } else {
        m_quickStopAccumulator = 0.0;
      }
    }

    double slewedXSpeed = -m_speedLimiter.calculate(xSpeed) * DriveConstants.kMaxSpeed;
    //double slewedRotation = -m_rotLimiter.calculate(angularPower) * DriveConstants.kMaxAngularSpeed;

    //m_driveTrain.arcadeDrive(slewedXSpeed, slewedRotation);
    m_driveTrain.arcadeDrive(slewedXSpeed, -angularPower * DriveConstants.kMaxAngularSpeed);
    // double leftMotorOutput = xSpeed + angularPower;
    // double rightMotorOutput = xSpeed - angularPower;

    // If rotation is overpowered, reduce both outputs to within acceptable range
    // if (overPower) {
    //   if (leftMotorOutput > 1.0) {
    //     rightMotorOutput -= leftMotorOutput - 1.0;
    //     leftMotorOutput = 1.0;
    //   } else if (rightMotorOutput > 1.0) {
    //     leftMotorOutput -= rightMotorOutput - 1.0;
    //     rightMotorOutput = 1.0;
    //   } else if (leftMotorOutput < -1.0) {
    //     rightMotorOutput -= leftMotorOutput + 1.0;
    //     leftMotorOutput = -1.0;
    //   } else if (rightMotorOutput < -1.0) {
    //     leftMotorOutput -= rightMotorOutput + 1.0;
    //     rightMotorOutput = -1.0;
    //   }
    // }

    // // Normalize the wheel speeds
    // double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
    // if (maxMagnitude > 1.0) {
    //   leftMotorOutput /= maxMagnitude;
    //   rightMotorOutput /= maxMagnitude;
    // }

    // m_leftMotor.set(leftMotorOutput * m_maxOutput);
    // m_rightMotor.set(rightMotorOutput * m_maxOutput * m_rightSideInvertMultiplier);

    // feed();
  }

  private void arcadeDrive(double xSpeed, double zRotation) {
    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    xSpeed = applyDeadband(xSpeed, kDefaultDeadband);

    zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
    zRotation = applyDeadband(zRotation, kDefaultDeadband);

    // Square the inputs (while preserving the sign) to increase fine control
    // while permitting full power.
    if (kSquareInputs) {
      xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
      zRotation = Math.copySign(zRotation * zRotation, zRotation);
    }

    double slewedXSpeed = -m_speedLimiter.calculate(xSpeed) * DriveConstants.kMaxSpeed;
    double slewedRotation = -m_rotLimiter.calculate(zRotation) * DriveConstants.kMaxAngularSpeed;

    m_driveTrain.arcadeDrive(slewedXSpeed, slewedRotation);
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

    /**
   * Returns 0.0 if the given value is within the specified range around zero. The remaining range
   * between the deadband and 1.0 is scaled from 0.0 to 1.0.
   *
   * @param value    value to clip
   * @param deadband range around zero
   */
  protected double applyDeadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }
}