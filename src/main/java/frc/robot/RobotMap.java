/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  //slide
  public static int slideCanId(){
    return 3;
  }
  //drive train
  public static int frontLeftMotorCanId() {
    return 11;
  }

  public static int backLeftMotorCanId() {
    return 10;
  }

  public static int frontRightMotorCanId() {
    return 21;
  }

  public static int backRightMotorCanId() {
    return 20;
  }
  //lift
  public static int leftLiftMotorCanId() {
    return 0; //replace with actual can id
  }
  public static int rightLiftMotorCanId() {
    return 0; //replace with actual can id
  }

  public static int grabberCanId(){
    return 0;
  
  }

  public static int armCanId(){
    return 0;
  }

  public static int pmcArmCanID = 5;
  public static int ArmSolenoidForward = 2;
  public static int ArmSolenoidReverse =  3;
  


  

}