/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * This command drives the robot over a given distance with simple proportional control.
 * This command will drive a given distance limiting to a maximum speed.
 */
public class DriveTrainMove extends Command {
  private double driveForwardSpeed;
  private double distance;
  private double error;
  private final double kTolerance = 0.1;
  private final double kP = -1.0 / 5.0; 
  
  public DriveTrainMove() {
    this(10, 0.5);
  }

  public DriveTrainMove(double dist) {
    this(dist, 0.5);
  }

  public DriveTrainMove(double dist, double maxSpeed) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_drivetrain);
    distance = dist;
    driveForwardSpeed = maxSpeed;
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_drivetrain.getRightEncoder().reset();
    setTimeout(2);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_drivetrain.update(Robot.m_oi.getForwardValue(), Robot.m_oi.getTurnValue());
    error = (distance - Robot.m_drivetrain.getRightEncoder().getDistance());
    if (driveForwardSpeed * kP * error >= driveForwardSpeed) {
      Robot.m_drivetrain.arcadeDrive(driveForwardSpeed, driveForwardSpeed);
    } else {
      Robot.m_drivetrain.arcadeDrive(driveForwardSpeed * kP * error, driveForwardSpeed * kP * error);
    }
    //need to add this command to continuously run in the scheduler. Trigger with a joystick input or just with start of auto/teleop?
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Math.abs(error) <= kTolerance) || isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_drivetrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}