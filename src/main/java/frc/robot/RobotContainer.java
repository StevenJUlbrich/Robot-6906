/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //Intake Subsystem and Commands
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem);

  //pneumatics Subsystem and Commands
  PneumaticsSubsystem pneumaticsSubsystem = new PneumaticsSubsystem();
  PneumaticsExtendPistonCommand pneumaticsExtendPistonCommand = new PneumaticsExtendPistonCommand(pneumaticsSubsystem);
  PneumaticsRetractPistonCommand pneumaticsRetractPistonCommand = new PneumaticsRetractPistonCommand(pneumaticsSubsystem);

  //Shooter Subsystem and Command
  Shooter shooterSubsystem = new Shooter(); //subsystem
  ShooterLongshotCommand shooterLongshotCommand = new ShooterLongshotCommand(shooterSubsystem);
  ShooterMediumshotCommand shooterMediumshotCommand = new ShooterMediumshotCommand(shooterSubsystem);
  ShooterShortshotCommand shooterShortshotCommand = new ShooterShortshotCommand(shooterSubsystem);
  ShooterDumpshotCommand shooterDumpshotCommand = new ShooterDumpshotCommand(shooterSubsystem);

  //ActivateFeederCommand
  ActivateFeederCommand activateFeederCommand = new ActivateFeederCommand(shooterSubsystem);

  //private final DriveTrain m_drive = new DriveTrain();
  public static XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  public static XboxController m_otherController = new XboxController(OIConstants.kOtherControllerPort);

  //private final AutonomousCommand m_autonomousCommand = new AutonomousCommand(m_drive, m_shooter);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

  }

  public Shooter getShooterSubsystem() {return shooterSubsystem;}

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //m_drive.arcadeDrive(m_driverController.getY(Hand.kRight), m_driverController.getX(Hand.kLeft));
     //Intake Subsystem and Commands
    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem);

    final JoystickButton activateIntakeButton = new JoystickButton(m_driverController, 2);
    activateIntakeButton.whenHeld(intakeCommand);

    //X for down
    //y up
    //Map buttons 3 and 4 on the controller to extend and retract the piston, respectively:
    final JoystickButton extendPistonButton = new JoystickButton(m_driverController, 3);
    final JoystickButton retractPistonButton = new JoystickButton(m_driverController, 4);

    //Define each button's behaviour:
    extendPistonButton.whenHeld(pneumaticsExtendPistonCommand);
    retractPistonButton.whenHeld(pneumaticsRetractPistonCommand);

    //---- Shooter mappings and setting the values ----

    //High = B, Medium = A, Low = X
    final JoystickButton longshotButton = new JoystickButton(m_otherController, Constants.Y);
    final JoystickButton mediumshotButton = new JoystickButton(m_otherController, Constants.B);
    final JoystickButton shortshotButton = new JoystickButton(m_otherController, Constants.A);
    final JoystickButton dumpshotButton = new JoystickButton(m_otherController, Constants.X);

    longshotButton.whenHeld(shooterLongshotCommand);
    mediumshotButton.whenHeld(shooterMediumshotCommand);
    shortshotButton.whenHeld(shooterShortshotCommand);
    dumpshotButton.whenHeld(shooterDumpshotCommand);
  }

  /**
   * A simple getter method for the drivetrain system
   * 
   * @return m_drive
   */
  // public DriveTrain getDrivetrain() {
  //   return m_drive;
  // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   return m_autonomousCommand;
  // }
}