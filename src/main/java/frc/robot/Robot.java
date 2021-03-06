/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ActivateFeederCommand;
import frc.robot.subsystems.*;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private static  RobotContainer m_robotContainer;
  private static DriveTrain m_drive = new DriveTrain();
  private static Shooter m_shooter;
  public static XboxController m_otherController = new XboxController(OIConstants.kOtherControllerPort);
  public Shooter shooterSubsystem;
  public ActivateFeederCommand activateFeederCommand;

  final JoystickButton activateFeederButton = new JoystickButton(m_otherController, 6);



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    shooterSubsystem = m_robotContainer.getShooterSubsystem();
    activateFeederCommand = new ActivateFeederCommand(shooterSubsystem);
    //m_drive = new DriveTrain();

    
    
  }
    /**
   * A simple getter method for RobotContainer.java
   * 
   * @return m_robotContainer
   */
  public static RobotContainer getRobotContainer() {
    return m_robotContainer;
  }

  /**
   * A simple getter method for Drivetrain.java
   * 
   * @return m_drive
   */
  public static DriveTrain getDrivetrain() {
    //return m_drive;
    return null;
  }

      /**
   * A simple getter method for Shooter.java
   * 
   * @return m_shooter
   */
  public static Shooter getShooter() {
    return m_shooter;
  }
  
  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    m_drive.arcadeDrive(RobotContainer.m_driverController.getY(Hand.kRight) * -1, RobotContainer.m_driverController.getX(Hand.kLeft));
    System.out.println("Y value: " + RobotContainer.m_driverController.getY(Hand.kRight));
    System.out.println("X value: " + RobotContainer.m_driverController.getX(Hand.kLeft));

    if( activateFeederButton.get()){
      activateFeederCommand.execute();
    }
    else{
      activateFeederCommand.cancel();
    }

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
