/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;

public class PneumaticsExtendPistonCommand extends CommandBase {

  //Member variables will be subsystems that the command will be executed on:
  PneumaticsSubsystem pneumaticsSubsystem;

  //Pass in an instance of all the required subsystems in the constructor.
  //and set them to the member variables.
  public PneumaticsExtendPistonCommand(PneumaticsSubsystem pneumatics) {
    // Use addRequirements() here to declare subsystem dependencies.
    pneumaticsSubsystem = pneumatics;
    addRequirements(pneumaticsSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pneumaticsSubsystem.extendPiston(); //This is the command to execute -- extend the 
    System.out.println("Extending Piston!");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
