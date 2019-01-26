/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.SlideForward;
import frc.robot.commands.SlideStop;
import frc.robot.RobotMap;
import frc.robot.Robot;

import java.lang.Math;
/**
 * Add your docs here.
 */
public class Slide extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private WPI_TalonSRX m_slide;
  public Slide(){

    m_slide = new WPI_TalonSRX(RobotMap.slideCanId());
  }
  
  public void slideOn(){
    //if rightTrigger > leftTrigger, move slide right. If rightTrigger < leftTrigger, move slide left.
    //if neither (both are equal), no movement.
    if (Math.abs(Robot.m_oi.getRightSlideValue()) > Math.abs(Robot.m_oi.getLeftSlideValue())) {
      m_slide.set(Robot.m_oi.getRightSlideValue());
    } else if (Math.abs(Robot.m_oi.getRightSlideValue()) < Math.abs(Robot.m_oi.getLeftSlideValue())) {
      m_slide.set(Robot.m_oi.getLeftSlideValue());
    } else {
      m_slide.set(0);
    }

  }

  public void slideStop(){

    m_slide.set(0);

  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
