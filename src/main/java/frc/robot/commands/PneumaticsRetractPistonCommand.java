/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;

public class PneumaticsRetractPistonCommand extends CommandBase {

  private PneumaticsSubsystem pneumaticsSubsystem /*= new PneumaticsSubsystem()*/;

  /**
   * Creates a new PneumaticsRetractPistonCommand.
   */
  public PneumaticsRetractPistonCommand(PneumaticsSubsystem pneumaticsSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pneumaticsSubsystem = pneumaticsSubsystem;
    addRequirements(this.pneumaticsSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pneumaticsSubsystem.retractPiston();
    System.out.println("Retracting Piston!");
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
