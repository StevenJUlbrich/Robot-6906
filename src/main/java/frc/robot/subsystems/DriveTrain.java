/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveTrain extends SubsystemBase {

   public static final CANSparkMax m_leftMaster = new CANSparkMax(DriveConstants.leftMotorMasterID, MotorType.kBrushless);
   public static final CANSparkMax m_leftSlave = new CANSparkMax(DriveConstants.leftMotorSlaveID, MotorType.kBrushless);
   public static final CANSparkMax m_rightMaster = new CANSparkMax(DriveConstants.leftMotorMasterID, MotorType.kBrushless);
   public static final CANSparkMax m_rightSlave = new CANSparkMax(DriveConstants.leftMotorSlaveID, MotorType.kBrushless);
   public static  SpeedController leftSideGroup;
   public static  SpeedController rightSideGroup;
   public static DifferentialDrive drive;

    public DriveTrain() {

        //Reset NEO Spark Controllers to Factory Defaults
        m_leftMaster.restoreFactoryDefaults();
        m_leftSlave.restoreFactoryDefaults();
        m_rightMaster.restoreFactoryDefaults();
        m_rightSlave.restoreFactoryDefaults();


        



        
    






    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}