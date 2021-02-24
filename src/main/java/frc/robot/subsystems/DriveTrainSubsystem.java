/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

// import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.GyroProvider;
import frc.robot.util.SparkMaxProvider;
// import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;

public class DriveTrainSubsystem extends SubsystemBase {


  private final CANSparkMax m_leftMaster = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
  private final CANSparkMax m_leftFollower = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);
  private final CANSparkMax m_rightMaster = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
  private final CANSparkMax m_rightFollower = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);

  private final CANEncoder m_leftEncoder = m_leftMaster.getEncoder();
  private final CANEncoder m_rightEncoder = m_rightMaster.getEncoder();

  private final CANPIDController m_leftMasterPIDController = m_leftMaster.getPIDController();
  private final CANPIDController m_rightMasterPIDController = m_rightMaster.getPIDController();

  private final GyroProvider m_gyro;

  private final DifferentialDriveKinematics m_kinematics
      = new DifferentialDriveKinematics(DriveConstants.kTrackWidth);

  private final DifferentialDriveOdometry m_odometry;

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter);

  // The encoderConstant is the factor that converts the angular units given by
  // the encoder to the angular units covered by the wheels.
  private final double encoderConstant = (1 / DriveConstants.kDriveGearing) * DriveConstants.kWheelDiameter * Math.PI;

 // The robot's drive  

  public DriveTrainSubsystem(SparkMaxProvider speedControllerProvider, GyroProvider gyroProvider) {
    m_gyro = gyroProvider;
    gyroProvider.reset();

    m_leftMaster.restoreFactoryDefaults();
    m_rightMaster.restoreFactoryDefaults();

    m_leftMaster.setInverted(false);
    m_leftMaster.setIdleMode(IdleMode.kBrake);
    m_rightMaster.setInverted(true);
    m_rightMaster.setIdleMode(IdleMode.kBrake);

    m_leftFollower.setIdleMode(IdleMode.kBrake);
    m_leftFollower.follow(m_leftMaster);
    m_rightFollower.setIdleMode(IdleMode.kBrake);
    m_rightFollower.follow(m_rightMaster);

    m_leftEncoder.setVelocityConversionFactor(encoderConstant * 0.0166666666666667);
    m_rightEncoder.setVelocityConversionFactor(encoderConstant * 0.0166666666666667);
    m_leftEncoder.setPositionConversionFactor(encoderConstant);
    m_rightEncoder.setPositionConversionFactor(encoderConstant);

    m_leftMaster.setSmartCurrentLimit(DriveConstants.kCurrentLimit);
    m_rightMaster.setSmartCurrentLimit(DriveConstants.kCurrentLimit);
    m_leftFollower.setSmartCurrentLimit(DriveConstants.kCurrentLimit);
    m_rightFollower.setSmartCurrentLimit(DriveConstants.kCurrentLimit);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(getAngle());
    
  }

  public void tankDrive(double left, double right) {
    // m_differentialDrive.tankDrive(left, right);
    setSpeeds(left*DriveConstants.kMaxSpeed, right*DriveConstants.kMaxSpeed);
  }

  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public double getPoseRotation() {
    return m_odometry.getPoseMeters().getRotation().getDegrees();
  }

  public void setPose(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, getAngle());
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(m_gyro.getHeading());
  }

  public void stop() {
    m_leftMaster.set(0);
    m_rightMaster.set(0);
  }

  public double getLeftEncoderPosition() {
    return m_leftEncoder.getPosition();
  }
  public double getLeftEncoderVelocity() {
    return m_leftEncoder.getVelocity();
  }
  public double getRightEncoderPosition() {
    return m_rightEncoder.getPosition();
  }
  public double getRightEncoderVelocity() {
    return m_rightEncoder.getVelocity();
  }

  public void setSpeeds(double left, double right) {
    setSpeeds(new DifferentialDriveWheelSpeeds(left, right));
  }

   /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    m_leftMasterPIDController.setReference(speeds.leftMetersPerSecond, ControlType.kVelocity, 
                                            /* PID Slot */ 0, leftFeedforward, ArbFFUnits.kVoltage);
    m_rightMasterPIDController.setReference(speeds.rightMetersPerSecond, ControlType.kVelocity, 
                                            /* PID Slot */ 0, rightFeedforward, ArbFFUnits.kVoltage);
    
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot    Angular velocity in rad/s.
   */
  @SuppressWarnings("ParameterName")
  public void arcadeDrive(double xSpeed, double rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));

    setSpeeds(wheelSpeeds);
  }

  public void directDrive(double xSpeed, double rot) {
    arcadeDrive(xSpeed, rot);
  }

  public void directDrive(ChassisSpeeds chassisSpeeds) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(chassisSpeeds);

    setSpeeds(wheelSpeeds);

  }

    /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }


  public DifferentialDriveKinematics getDriveKinematics() {
    return m_kinematics;
  }

  @Override
  public void periodic() {
    // Use these to verify the conversion factors are correct
    SmartDashboard.putNumber("Left Encoder Position", getLeftEncoderPosition());
    SmartDashboard.putNumber("Right Encoder Position", getRightEncoderPosition());
    SmartDashboard.putNumber("Left Encoder Velocity", getLeftEncoderVelocity());
    SmartDashboard.putNumber("Right Encoder Velocity", getRightEncoderVelocity());

    SmartDashboard.putNumber("Pose X", m_odometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("Pose Y", m_odometry.getPoseMeters().getTranslation().getY());
    SmartDashboard.putNumber("Pose Rotation", m_odometry.getPoseMeters().getRotation().getDegrees());
    
    updateOdometry();    
  }

  public void updateOdometry() {
    m_odometry.update(getAngle(), getLeftEncoderPosition(), getRightEncoderPosition());
  }
    /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
    public void setMaxOutput(double maxOutput) {
        m_leftMasterPIDController.setOutputRange(maxOutput, maxOutput, 0);
        m_rightMasterPIDController.setOutputRange(maxOutput, maxOutput, 0);
    }
}
