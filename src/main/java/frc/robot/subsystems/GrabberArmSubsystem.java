/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;


/**
 * Add your docs here.
 */
public class GrabberArmSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
    private DoubleSolenoid m_grabberarm;

    @Override
    public void initDefaultCommand() {
      m_grabberarm = new DoubleSolenoid(RobotMap.pmcArmCanID,RobotMap.ArmSolenoidForward,RobotMap.ArmSolenoidReverse);
      m_grabberarm.set(DoubleSolenoid.Value.kOff);
        
    }

 
   


   

    public void turnOff() {
        m_grabberarm.set(DoubleSolenoid.Value.kOff);

    }

    public void extend() {
      m_grabberarm.set(DoubleSolenoid.Value.kForward);

    }

    public void retract(){
      m_grabberarm.set(DoubleSolenoid.Value.kReverse);

    }

   
    








}
