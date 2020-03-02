/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.DriveTrain;
//import frc.robot.subsystems.Launcher;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Auto_MoveFwd_1meter;
//import frc.robot.commands.Auto_Move_S_curve;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  DriveTrain m_robotDrive;
  //Launcher m_launcher;
  Auto_MoveFwd_1meter m_autoCommand;
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_otherController = new XboxController(OIConstants.kOtherControllerPort);


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_robotDrive = new DriveTrain();
    m_autoCommand = new Auto_MoveFwd_1meter(m_robotDrive);
    //m_launcher = new Launcher();
    // m_autoCommand = new Auto_Move_S_curve(m_robotDrive);
    // Configure the button bindings
    configureButtonBindings();

    m_robotDrive.setDefaultCommand(
    new RunCommand(() -> m_robotDrive
      .arcadeDrive(m_driverController.getY(GenericHID.Hand.kLeft),
        m_driverController.getX(GenericHID.Hand.kRight)), m_robotDrive));
    
        /**
        m_launcher.setDefaultCommand(
          new RunCommand(() -> m_launcher
          .launch(m_otherController.getY(GenericHID.Hand.kLeft)), m_launcher)
        );
         */
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand.getAutoCommand();
  }
}