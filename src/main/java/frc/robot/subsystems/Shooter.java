/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private VictorSPX indexMotor = new VictorSPX(7);
  private CANSparkMax m_motorTop;
  private CANSparkMax m_motorBottom;
  private CANPIDController m_pidControllerTop;
  private CANPIDController m_pidControllerBottom;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    m_motorTop = new CANSparkMax(ShooterConstants.topMotorID, MotorType.kBrushless);
    m_motorBottom = new CANSparkMax(ShooterConstants.bottomMotorID, MotorType.kBrushless);

    m_motorTop.restoreFactoryDefaults();
    m_motorBottom.restoreFactoryDefaults();

    m_motorTop.setIdleMode(IdleMode.kCoast);
    m_motorBottom.setIdleMode(IdleMode.kCoast);

    m_motorTop.setInverted(true);

    /**
     * In order to use PID functionality for a controller, a CANPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidControllerTop = m_motorTop.getPIDController();
    m_pidControllerBottom = m_motorBottom.getPIDController();

    m_motorTop.getEncoder();
    m_motorBottom.getEncoder();

    //setup PID Controller Top Motor
    m_pidControllerTop.setP(ShooterConstants.kP);
    m_pidControllerTop.setI(ShooterConstants.kI);
    m_pidControllerTop.setD(ShooterConstants.kD);
    m_pidControllerTop.setIZone(ShooterConstants.kIz);
    m_pidControllerTop.setFF(ShooterConstants.kFF);
    m_pidControllerTop.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);


    //setup PID Controller Bottom Motor
//setup PID Controller Top Motor
    m_pidControllerBottom.setP(ShooterConstants.kP);
    m_pidControllerBottom.setI(ShooterConstants.kI);
    m_pidControllerBottom.setD(ShooterConstants.kD);
    m_pidControllerBottom.setIZone(ShooterConstants.kIz);
    m_pidControllerBottom.setFF(ShooterConstants.kFF);
    m_pidControllerBottom.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);
  }

public void set(Double motorTopMaxRPM, Double motorBottomMaxRPM){
  m_pidControllerTop.setReference(motorTopMaxRPM, ControlType.kVelocity);
  m_pidControllerBottom.setReference(motorBottomMaxRPM, ControlType.kVelocity);
}

public void activateFeeder() {
  indexMotor.set(ControlMode.PercentOutput, 5.0);
  // m_pidControllerTop.setReference(Constants.LONG_SHOT_TOP_MOTOR, ControlType.kVelocity);
  // m_pidControllerBottom.setReference(Constants.LONG_SHOT_BOTTOM_MOTOR, ControlType.kVelocity);
}

public void stopFeeder() {
  indexMotor.set(ControlMode.PercentOutput, 0.0);
}

public void stop(){
  m_pidControllerTop.setReference(0, ControlType.kVelocity);
  m_pidControllerBottom.setReference(0, ControlType.kVelocity);
  indexMotor.set(ControlMode.PercentOutput, 0.0);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
