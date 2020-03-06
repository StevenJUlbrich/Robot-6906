/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticsSubsystem extends SubsystemBase {

  /*
  The Pneumatics Control Module (PCM) we will be using has
  been given the default CAN id of 0. This means we do not
  have to specify an id in the DoubleSolenoid constructor.

  In the DoubleSolenoid constructor, we only need to specify
  the port numbers on which the double solenoid is connected
  to the PCM (in our case, 1 and 2).
  */
  private DoubleSolenoid doubleSolenoid;

  /**
   * Creates a new PneumaticsSubsystem.
   */
  public PneumaticsSubsystem() {
    super(); //add this
    try {
      doubleSolenoid = new DoubleSolenoid(0, 1);
    } catch(Exception e) {
      e.printStackTrace();
    }
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void extendPiston() {
    //Should extend the piston
    doubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retractPiston() {
    //Should retract the piston
    doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
}
