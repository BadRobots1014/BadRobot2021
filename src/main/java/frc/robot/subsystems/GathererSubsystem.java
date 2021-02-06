/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GathererSubsystem extends SubsystemBase {
  private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(0, 1);
  public static final double kGathererSpeed = .5;

  private final TalonSRX m_gatherer;

  private BooleanSupplier m_driverTrigger = () -> false;

  /**
   * Creates a new ExampleSubsystem.
   */
  public GathererSubsystem(TalonSRX talon) {
    m_gatherer = talon;
    m_gatherer.setInverted(true);

  }

  public void setTriggerSupplier(BooleanSupplier trigger) {
    m_driverTrigger = trigger;
  }

  public boolean isGathererOut() {
    boolean gathererState = m_doubleSolenoid.get() == Value.kReverse;
    //System.out.println("Gatherer is out:" + gathererState);
    return gathererState;
  }

  public void gathererOut() {
    System.out.println("Gather out");
    m_doubleSolenoid.set(Value.kReverse);
  }

  public void gathererIn() {
    System.out.println("Gather in");
    m_doubleSolenoid.set(Value.kForward);
  }

  public void gathererToggle() {
    if (m_doubleSolenoid.get() == Value.kReverse) {
      gathererOut();
    } else if (m_doubleSolenoid.get() == Value.kForward) {
      gathererIn();
    } else {
      gathererOut();
    }
  }

  public void runGatherer() { // Run gatherer now only works if the gatherer is out.
                              // We only ever want to run the gatherer while it is out
    if (isGathererOut() && !m_driverTrigger.getAsBoolean()) {
      m_gatherer.set(ControlMode.PercentOutput, kGathererSpeed);
    } else if (isGathererOut() && m_driverTrigger.getAsBoolean()) {
      m_gatherer.set(ControlMode.PercentOutput, -kGathererSpeed);
    } else {
      m_gatherer.set(ControlMode.PercentOutput, 0);
    }
  }

  public void runGathererReversed() { // Run gatherer now only works if the gatherer is out.
    if (isGathererOut()) {
      m_gatherer.set(ControlMode.PercentOutput, -kGathererSpeed);
    } else {
      m_gatherer.set(ControlMode.PercentOutput, 0);
    }
  }

  // public void runGathererReversed() {
  //   m_gatherer.set(ControlMode.PercentOutput, -kGathererSpeed);
  // }

  public void stopGather() {
    m_gatherer.set(ControlMode.PercentOutput, 0);
  }
}
