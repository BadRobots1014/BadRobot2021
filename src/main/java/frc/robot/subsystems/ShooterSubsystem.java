/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;


public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX m_shooterMotor = new TalonFX(ShooterConstants.kShooterId);
  private final Servo m_hoodServo = new Servo(0);

  public final double kDeadband = 0.05;
  private DoubleSupplier m_joystickSupplier = () -> 0.0;
  private BooleanSupplier m_triggerSupplier = () -> false;

  private final ShuffleboardTab m_shooterTab = Shuffleboard.getTab("Shooting");
  private final NetworkTableEntry m_velocityEntry = m_shooterTab.add("Shooter Velocity", 0).getEntry();
  private final NetworkTableEntry m_currentEntry = m_shooterTab.add("Shooter Current", 0).getEntry();
  private final NetworkTableEntry m_deltaVelocityEntry = m_shooterTab.add("Delta Velocity", 0).getEntry();
  
  /**
   * Creates a new ExampleSubsystem.
   */
  public ShooterSubsystem() {
    m_shooterMotor.setNeutralMode(NeutralMode.Coast);
    m_shooterMotor.configClosedloopRamp(4);
    m_shooterMotor.setInverted(ShooterConstants.kShooterReversed);
    m_shooterMotor.config_kF(0, ShooterConstants.kF);
    m_shooterMotor.config_kP(0, ShooterConstants.kP);
  }

  /**
   * Creates controller bindings
   * @param joystickSupplier Supplies the percent output the magazine motor runs at
   * @param triggerSupplier Supplies the boolean which determines whether or not the value from the joystick is passed
   */
  public void setJoystickSupplier(DoubleSupplier joystickSupplier, BooleanSupplier triggerSupplier) {
    m_joystickSupplier = joystickSupplier;
    m_triggerSupplier = triggerSupplier;
  }
  
  public void extendHood() {
    m_hoodServo.setAngle(67);
  }

  public void retractHood() {
    m_hoodServo.setAngle(0);
  }

  public void runShooter() {
    m_shooterMotor.set(TalonFXControlMode.Velocity, ShooterConstants.kDesiredAngularSpeed);
  }

  public void controlShooter() {
    if (m_triggerSupplier.getAsBoolean()) {
      m_shooterMotor.set(TalonFXControlMode.PercentOutput, m_joystickSupplier.getAsDouble());
    } else {
      m_shooterMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
  }

  public void stopShooter() {
    m_shooterMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

  public double getPosition(){
    return m_hoodServo.getPosition();
  }

  public void updateEntries() {
    m_velocityEntry.setDouble(m_shooterMotor.getSelectedSensorVelocity());
    m_currentEntry.setDouble(m_shooterMotor.getStatorCurrent());
    m_deltaVelocityEntry.setDouble(m_velocityEntry.getDouble(ShooterConstants.kDesiredAngularSpeed) - ShooterConstants.kDesiredAngularSpeed);
  }

  // Return the difference between the desired velocity and the current velocity
  // Guages what's going on with the motor.
  public double getDeltaDesiredVelocity() {
    return m_velocityEntry.getDouble(ShooterConstants.kDesiredAngularSpeed) - ShooterConstants.kDesiredAngularSpeed;
  }

  public double getDeltaDesiredActiveCurrent() {
    return m_currentEntry.getDouble(ShooterConstants.kDesiredActiveCurrent) - ShooterConstants.kDesiredActiveCurrent;
  }
  
  @Override
  public void periodic() {
    updateEntries();
  }
  
  /* Once specifics are available
  public void setDesiredVelocity() {
    m_shooterMotor.set(TalonFXControlMode.Velocity, ShooterConstants.kDesiredAngularSpeed * ShooterConstants.kEncoderConstant);
  }
  */

  /**
   * Pertinent information:
   * Encoder: 2048 CPR
   * 1 rpm -> 1.462847143 encoder velocity units
   * Gearing ratio of wheel to motor: (7/3):1
   */

  /**
   * Shooter flow:
   * 
   * Mode 1: Single fire
   * 1. Accelerate the shooter wheel to the desired speed
   * 2. Activate the feeder
   * 3. Wait for the ball to go through--decrease in angular velocity and current spike
   * 4. Chill
   * 
   * Mode 2: Continuous fire
   * 
   */

}
