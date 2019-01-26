/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDSourceType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // create CAN Talon SRX objects
  private WPI_TalonSRX m_frontleft;
  private WPI_TalonSRX m_backleft;
  private WPI_TalonSRX m_frontright;
  private WPI_TalonSRX m_backright;
  //create drive train and drive train sides objects
  private SpeedControllerGroup m_left;
  private SpeedControllerGroup m_right;
  private DifferentialDrive m_drive;
  //create encoders
  private Encoder rightEncoder = new Encoder(1, 2, true, EncodingType.k4X);
  private Encoder leftEncoder = new Encoder(3, 4, false, EncodingType.k4X);

  public DriveTrain(){
    //initialize + set objects created above
    m_frontleft = new WPI_TalonSRX(RobotMap.frontLeftMotorCanId());
    m_backleft = new WPI_TalonSRX(RobotMap.backLeftMotorCanId());
    m_left = new SpeedControllerGroup(m_frontleft, m_backleft);

    m_frontright = new WPI_TalonSRX(RobotMap.frontRightMotorCanId());
    m_backright = new WPI_TalonSRX(RobotMap.backRightMotorCanId());
    m_right = new SpeedControllerGroup(m_frontright, m_backright);
    //m_left.setInverted(true); invert left side
    //Configure the RobotDrive to reflect the fact that all our motors are
    //Wired backwards and our drivers sensitivity preferences.

    m_drive = new DifferentialDrive(m_left, m_right);

    m_drive.setSafetyEnabled(true);
    m_drive.setExpiration(0.1);
    m_drive.setSensitivity(0.5);
    m_drive.setMaxOutput(1.0);
    m_drive.setInvertedMotor(RobotDrive.MotorType.kFrontleft, true);
    m_drive.setInvertedMotor(RobotDrive.MotorType.kBackleft, true);
    m_drive.setInvertedMotor(RobotDrive.MotorType.kFrontright, true);
    m_drive.setInvertedMotor(RobotDrive.MotorType.kBackright, true);

    //Configure encoders
    rightEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
    leftEncoder.setPIDSourceType(PIDSourceType.kDisplacement);

    if (Robot.isReal()) { // Converts to feet
      rightEncoder.setDistancePerPulse(0.0785398);
      leftEncoder.setDistancePerPulse(0.0785398);
    } else { // Convert to feet 4in diameter wheels with 360 tick stimulated
      // encoders
      rightEncoder.setDistancePerPulse(
        (4.0/* in */ * Math.PI) / (360.0 * 12.0/* in/ft */));
      leftEncoder.setDistancePerPulse(
          (4.0/* in */ * Math.PI) / (360.0 * 12.0/* in/ft */));
    }
    
    /**
     * @ param joy
     * Xbox style joystick to use as the input for arcade drive.
     */
    public void arcadeDrive(Joystick joy) {
      drive.arcadeDrive(joy.getY(), joy.getRawAxis(4));
    }

    SpeedControllerGroup leftMotors = new
    SpeedControllerGroup(m_frontleft, m_backleft);

    SpeedControllerGroup rightMotors = new
    SpeedControllerGroup (m_frontright, m_backright);
  }
  public void aracdeDrive(double moveSpeed, double rotateSpeed) {
    differentialDrive.arcadeDrive(moveSpeed, rotateSpeed);
  }

  public void update(double y, double z){
    m_drive.arcadeDrive(y, z);
  }


  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveWithJoystick());
  }

  /**
   * @param joy
  * Xbox style joystick to use as the input for arcade drive.
  */
  public void arcadeDrive(Joystick joy) {
    drive.arcadeDrive(joy.getY(), joy.getRawAxis(4));
  }

/**
 * @return The encoder getting the distance and speed of left side of the drivetrain.
 */
public Encoder getLeftEncoder() {
  return leftEncoder;
}

/**
 * @return The encoder getting the distance and speed of right side of the drivetrain.
 */
public Encoder getRightEncoder() {
  return rightEncoder;
}

public void arcadeDrive(double moveSpeed, double rotateSpeed) {}
public void stop() {
  drive.arcadeDrive(0, 0);
}
}