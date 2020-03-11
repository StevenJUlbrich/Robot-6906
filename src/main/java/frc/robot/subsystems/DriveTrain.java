/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.Supplier;
import frc.robot.Constants.DriveConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.geometry.Pose2d;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;
//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableInstance;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.networktables.NetworkTable;

public class DriveTrain extends SubsystemBase {

  public CANSparkMax m_leftMaster;
  public CANSparkMax m_leftSlave;
  public CANSparkMax m_rightMaster;
  public CANSparkMax m_rightSlave;

  private final CANEncoder m_leftEncoder;
  private final CANEncoder m_rightEncoder;

  // AHRS GYRO
  private final AHRS m_gyro;

  // Odometry Class for Tracking Robot Pose;
  //private final DifferentialDriveOdometry m_odometry;

  Supplier<Double> gryoAngleRadians;
  DifferentialDrive m_drive;

  public DriveTrain() {

    // Define and Build Left Master NEO Motor
    m_leftMaster = new CANSparkMax(DriveConstants.leftMotorMasterID, MotorType.kBrushless);
    m_leftMaster.restoreFactoryDefaults();
    m_leftMaster.setInverted(DriveConstants.LeftInvertedBoolean);
    m_leftMaster.setIdleMode(DriveConstants.Idlemode);
    m_leftEncoder = m_leftMaster.getEncoder();
    m_leftEncoder.setPositionConversionFactor(DriveConstants.kNeoPositionConversionFactor);
    m_leftEncoder.setVelocityConversionFactor(DriveConstants.kNeoVelocityConversionFactor);

    // Define and Build Left Slave NEO Motor
    m_leftSlave = new CANSparkMax(DriveConstants.leftMotorSlaveID, MotorType.kBrushless);
    m_leftSlave.restoreFactoryDefaults();
    m_leftSlave.setIdleMode(DriveConstants.Idlemode);
    m_leftSlave.follow(m_leftMaster);

    // Define and Build Right Master NEO Motor
    m_rightMaster = new CANSparkMax(DriveConstants.rightMotorMasterID, MotorType.kBrushless);
    m_rightMaster.restoreFactoryDefaults();
    m_rightMaster.setInverted(DriveConstants.RightInvertedBoolean);
    m_rightMaster.setIdleMode(DriveConstants.Idlemode);
    m_rightEncoder = m_rightMaster.getEncoder();
    m_rightEncoder.setPositionConversionFactor(DriveConstants.kNeoPositionConversionFactor);
    m_rightEncoder.setVelocityConversionFactor(DriveConstants.kNeoVelocityConversionFactor);

    m_rightSlave = new CANSparkMax(DriveConstants.rightMotorSlaveID, MotorType.kBrushless);
    m_rightSlave.restoreFactoryDefaults();
    m_rightSlave.setIdleMode(DriveConstants.Idlemode);
    m_rightSlave.follow(m_rightMaster);

    m_drive = new DifferentialDrive(m_leftMaster, m_rightMaster);
    m_drive.setDeadband(0);

    resetEncoders();
    //m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    //
    // Configure gyro
    //

    // Note that the angle from the NavX and all implementors of wpilib Gyro
    // must be negated because getAngle returns a clockwise positive angle
    m_gyro = new AHRS(I2C.Port.kMXP);
    gryoAngleRadians = () -> -1 * Math.toRadians(m_gyro.getAngle());

  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_leftEncoder.setPosition(0.0);
    m_rightEncoder.setPosition(0.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Reset the robots sensors to the zero states.
   */
  public void reset() {
    m_gyro.reset();
    // m_leftEncoder.set;
    // m_rightEncoder.reset();
  }

  public void drive(final double left, final double right) {
    m_drive.tankDrive(left, right);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  /*public Pose2d getPose() {
    //return m_odometry.getPoseMeters();
  }*/

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update the odometry in the periodic block
    // Note that the setPositionConversionFactor() has been applied to the left and
    // right encoders in the constructor.
    //m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(final Pose2d pose) {
    resetEncoders();
    //m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  
  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(final double fwd, final double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }


  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(final double leftVolts, final double rightVolts) {
    // ToDo: Verify whether setVolatge() is applied to the follower motor
    // controller.
    m_leftMaster.setVoltage(leftVolts);
    m_rightMaster.setVoltage(-rightVolts);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    // Note that the position conversion factors have been applied to both the
    // encoders in the constructor
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public CANEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public CANEncoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(final double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  //IMU AHRS 
  public double getPitch() {
    return m_gyro.getPitch();
  }
  
  public double getYaw() {
    return m_gyro.getYaw();
  }
  public double getRoll() {
    return m_gyro.getRoll();
  }
  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
        //return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
        return 0;
  }

  public void stopMotors() {
    m_leftMaster.setVoltage(0);
    m_rightMaster.setVoltage(0);
  }

}