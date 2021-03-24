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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MagazineConstants;
import frc.robot.Constants.MagazineConstants.MagazineState;

public class MagazineSubsystem extends SubsystemBase {
  private final TalonSRX m_magazineMotor;

  private final DigitalInput m_inSensor = new DigitalInput(MagazineConstants.kInSensorPort);
  private final DigitalInput m_outSensor = new DigitalInput(MagazineConstants.kOutSensorPort);

  private boolean m_lastInSensorState = false;
  private boolean m_lastOutSensorState = false;
  private boolean m_lastShootTriggerState = false;
  private boolean m_lastReadyTriggerState = false;

  private final Timer m_shootingTimer = new Timer();

  private BooleanSupplier m_shootTriggerSupplier = () -> false;
  private BooleanSupplier m_readyTriggerSupplier = () -> false;

  private DoubleSupplier m_shooterVelocitySupplier = () -> 0.0;

  private MagazineState m_currentState = MagazineState.EMPTY;
  
  private final ShuffleboardTab m_shooterTab = Shuffleboard.getTab("Shooting");

  private final NetworkTableEntry m_currentStateEntry = m_shooterTab.add("Magazine state", "").getEntry();

  private final NetworkTableEntry m_inSensorState = m_shooterTab.add("Magazine input sensor state", true).getEntry();
  private final NetworkTableEntry m_outSensorState = m_shooterTab.add("Magazine output sensor state", true).getEntry();

  private final NetworkTableEntry m_ballCountEntry = m_shooterTab.add("Magazine ball count", 0)
                                                                 .withWidget(BuiltInWidgets.kTextView)
                                                                 .getEntry();

  private final int kShootCycleResetTimeout = 5;
  
  /**
   * Creates a new MagazineSubsystem.
   * @param magMotor The magazine motor object
   */
  public MagazineSubsystem(TalonSRX magMotor) {
    m_magazineMotor = magMotor;
    m_magazineMotor.setInverted(true);
  }

  /**
   * Creates controller bindings
   * @param shootTriggerSupplier Supplies the state of the shoot button
   * @param readyTriggerSupplier Supplies the state of the button to switch between ready/gather states
   */
  public void setJoystickSupplier(BooleanSupplier shootTriggerSupplier, BooleanSupplier readyTriggerSupplier) {
    m_shootTriggerSupplier = shootTriggerSupplier;
    m_readyTriggerSupplier = readyTriggerSupplier;
  }

  public void setShooterVelocitySupplier(DoubleSupplier shooterVelocitySupplier) {
    m_shooterVelocitySupplier = shooterVelocitySupplier;
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

  public boolean isShootingAllowed() {
    return (m_currentState == MagazineState.READY || m_currentState == MagazineState.SHOOT || m_currentState == MagazineState.FIRE);
  }

  public void runAuto() {
    boolean currentInSensorState = m_inSensor.get();
    boolean currentOutSensorState = m_outSensor.get();
    boolean currentShootTriggerState = m_shootTriggerSupplier.getAsBoolean();
    boolean currentReadyTriggerState = m_readyTriggerSupplier.getAsBoolean();

    switch (m_currentState) {
      case EMPTY:
        stopMotor();
        if (!m_lastInSensorState && currentInSensorState) {
          m_currentState = MagazineState.LOAD;
        }
        break;
      case LOAD:
        runMotor();
        if (!m_lastReadyTriggerState && currentReadyTriggerState) {
          m_currentState = MagazineState.READY;
        } else if (!m_lastOutSensorState && currentOutSensorState) {
          m_currentState = MagazineState.FULL;
        } else if (m_lastInSensorState && !currentInSensorState) {
          m_ballCountEntry.setNumber(m_ballCountEntry.getNumber(0).intValue() + 1);
          m_currentState = MagazineState.LOADED;
        }
        break;
      case LOADED:
        stopMotor();
        if (!m_lastReadyTriggerState && currentReadyTriggerState) {
          m_currentState = MagazineState.GOTO_READY;
        } else if (!m_lastOutSensorState && currentOutSensorState) {
          m_currentState = MagazineState.FULL;
        } else if (!m_lastInSensorState && currentInSensorState) {
          m_currentState = MagazineState.LOAD;
        }
        break;
      case FULL:
        stopMotor();
        m_currentState = MagazineState.READY;
        break;
      case READY:
        stopMotor();
        if (!m_lastShootTriggerState && currentShootTriggerState) {
          m_shootingTimer.reset();
          m_shootingTimer.start();
          m_currentState = MagazineState.SHOOT;
        } else if (!m_lastReadyTriggerState && currentReadyTriggerState) {
          m_currentState = MagazineState.LOADED;
        }
        break;
      case GOTO_READY:
        if (!m_lastOutSensorState && currentOutSensorState) {
          m_currentState = MagazineState.READY;
        } else {
          runMotor();
        }
        break;
      case SHOOT:
        stopMotor();
        if (m_lastShootTriggerState && !currentShootTriggerState) {
          m_shootingTimer.stop();
          if (m_shootingTimer.hasElapsed(kShootCycleResetTimeout)) {
            m_currentState = MagazineState.EMPTY;
          } else {
            m_currentState = MagazineState.READY;
          }
        }
        if (m_shooterVelocitySupplier.getAsDouble() > 10000) {
          m_currentState = MagazineState.FIRE;
        }
        break;
      case FIRE:
        runMotor();
        if (m_lastOutSensorState && !currentOutSensorState) {
          m_ballCountEntry.setNumber(m_ballCountEntry.getNumber(0).intValue() - 1);
        }
        if (m_lastShootTriggerState && !currentShootTriggerState) {
          m_shootingTimer.stop();
          if (m_shootingTimer.hasElapsed(kShootCycleResetTimeout)) {
            m_currentState = MagazineState.EMPTY;
          } else {
            m_currentState = MagazineState.READY;
          }
        }
        if (m_shooterVelocitySupplier.getAsDouble() < 10000) {
          m_currentState = MagazineState.SHOOT;
        }
    }

    m_lastInSensorState = currentInSensorState;
    m_lastOutSensorState = currentOutSensorState;
    m_lastShootTriggerState = currentShootTriggerState;
    m_lastReadyTriggerState = currentReadyTriggerState;
        
  }

  @Override
  public void periodic() {
    m_currentStateEntry.setString(m_currentState.name());
    m_inSensorState.setBoolean(m_inSensor.get());
    m_outSensorState.setBoolean(m_outSensor.get());
  }
}
