/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MagazineConstants;

public class MagazineSubsystem extends SubsystemBase {
  private final TalonSRX m_magazineMotor;
  private final DigitalInput m_inSensor = new DigitalInput(MagazineConstants.kInSensorPort);
  private final DigitalInput m_outSensor = new DigitalInput(MagazineConstants.kOutSensorPort);
  
  private final ShuffleboardTab m_shooterTab = Shuffleboard.getTab("Shooting");

  private final NetworkTableEntry m_inSensorState = m_shooterTab.add("Magazine input sensor state", true).getEntry();
  private final NetworkTableEntry m_outSensorState = m_shooterTab.add("Magazine output sensor state", true).getEntry();
  private final NetworkTableEntry m_ballCount = m_shooterTab.add("Magazine ball count", 0).getEntry();

  
  private DoubleSupplier m_joystickSupplier = () -> 0.0;
  private BooleanSupplier m_triggerSupplier = () -> false;
  
  /**
   * Creates a new MagazineSubsystem.
   */
  public MagazineSubsystem(TalonSRX magMotor) {
    m_magazineMotor = magMotor;
    m_magazineMotor.setInverted(true);
  }

  public void setJoystickSupplier(DoubleSupplier joystickSupplier, BooleanSupplier triggerSupplier) {
    m_joystickSupplier = joystickSupplier;
    m_triggerSupplier = triggerSupplier;
  }

  public void controlMagazine() {
    if (m_triggerSupplier.getAsBoolean()) {
      m_magazineMotor.set(ControlMode.PercentOutput, m_joystickSupplier.getAsDouble());
    } else {
      m_magazineMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  public void runMotor() {
    m_magazineMotor.set(ControlMode.PercentOutput, MagazineConstants.kMaxSpeed);
  }

  public void runMotorReversed() {
    m_magazineMotor.set(ControlMode.PercentOutput, -MagazineConstants.kMaxSpeed);
  }

  public void stopMotor() {
    m_magazineMotor.set(ControlMode.PercentOutput, 0);
  }

  public void runAuto() {
    if (!m_outSensor.get()) {
      runMotor();
    } else {
      stopMotor();
    }
  }

  // private boolean getInSensorState() { // true is unobstructed
  //   if (m_inSensor.getValue() > MagazineConstants.kSensorThreshold) {
  //     return true;
  //   } else {
  //     return false;
  //   }
  // }

  // private boolean getOutSensorState() { // true is unobstructed
  //   if (m_outSensor.getValue() > MagazineConstants.kSensorThreshold) {
  //     return true;
  //   } else {
  //     return false;
  //   }
  // }

  // private void updateBallCount() {
    
  //   if (m_inSensorState.getBoolean(true) == true && getInSensorState() == false) {
  //     m_ballCount.setDouble(m_ballCount.getDouble(0) + 1);
  //   }
    
  //   /*
  //   if (m_outSensorState.getBoolean(true) == true && getOutSensorState() == false) {
  //     m_ballCount.setDouble(m_ballCount.getDouble(0) - 1);
  //   }
  //   */
    
  //   m_inSensorState.setBoolean(getInSensorState());
  //   // m_shooterTab.add("Magazine output sensor state", getOutSensorState());
  // }

  @Override
  public void periodic() {
    // updateBallCount();
    // SmartDashboard.putBoolean("Magazine input sensor output", m_inSensor.get());
    SmartDashboard.putBoolean("Magazine output sensor output", m_outSensor.get());
    m_inSensorState.setBoolean(m_inSensor.get());
    m_outSensorState.setBoolean(m_outSensor.get());
    // This method will be called once per scheduler run
  }
}
