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
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDSourceType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import java.math.*;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // create TALON SRX objects
  private WPI_TalonSRX m_frontleft;
  private WPI_TalonSRX m_backleft;
  private WPI_TalonSRX m_frontright;
  private WPI_TalonSRX m_backright;
  private static final int TIMEOUT_MS = 1000; 
  private static double m_velocity_fps; // feet per second velocity
  private static double m_velocity_turn_rps; // rad per second
  private static double m_leftspeed;
  private static double m_rightspeed;
  private static final double WHEELBASE = 19.5/12; // inches on numerator, convert to feet
  private static double m_radius = WHEELBASE/2;

  private static int m_maxAmps = 0;
  private static double m_maxIntegral = 0;
  private static final double REVOLUTIONSPERFOOT = 1/Math.PI;
  private static final double COUNTSPERREVOLUTION = 4096;
  private static final double SECONDSPER100MILLISECONDS = .1;
  private static final double MAXVELOCITY = 5; // fps
  private static final double RAD_PER_DEG = Math.PI/180;

  private double m_p_right = 1.0;
  private double m_i_right = 0.001;
  private double m_d_right = 0.0;
  private double m_f_right = 0.1097;

  private double m_p_left = 1.0;
  private double m_i_left = 0.001;
  private double m_d_left = 0.0;
  private double m_f_left = 0.1097;

  public DriveTrain() {
    //initialize + set objects created above
    m_frontleft = new WPI_TalonSRX(Robot.m_map.getId(MapKeys.DRIVE_FRONTLEFT));
    m_backleft = new WPI_TalonSRX(Robot.m_map.getId(MapKeys.DRIVE_BACKLEFT));
    m_frontright = new WPI_TalonSRX(Robot.m_map.getId(MapKeys.DRIVE_FRONTRIGHT));
    m_backright = new WPI_TalonSRX(Robot.m_map.getId(MapKeys.DRIVE_BACKRIGHT));

    configTalons();
    m_velocity_fps = 0;
    m_velocity_turn_rps = 0;
    }

  public void configTalons() {
    /* first choose the sensor */
    m_frontright.selectProfileSlot(0, 0);
    m_frontright.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    m_frontright.setSensorPhase(true); // inverts the phase of the sensor if true.

    m_frontleft.selectProfileSlot(0, 0);
    m_frontleft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    m_frontleft.setSensorPhase(true);
  
	/* set closed loop gains in slot0 */
    m_frontright.config_kP(0, 0, TIMEOUT_MS);
    m_frontright.config_kI(0, 0, TIMEOUT_MS);
    m_frontright.config_kD(0, 0, TIMEOUT_MS);
    m_frontright.config_kF(0, 0, TIMEOUT_MS);

    m_frontleft.config_kP(0, 0, TIMEOUT_MS);
    m_frontleft.config_kI(0, 0, TIMEOUT_MS);
    m_frontleft.config_kD(0, 0, TIMEOUT_MS);
    m_frontleft.config_kF(0, 0, TIMEOUT_MS);

    m_frontright.configMaxIntegralAccumulator(0, m_maxIntegral, TIMEOUT_MS);
    m_frontright.configContinuousCurrentLimit(m_maxAmps, TIMEOUT_MS);
    m_frontright.configPeakCurrentLimit(m_maxAmps, TIMEOUT_MS);

    m_frontleft.configMaxIntegralAccumulator(0, m_maxIntegral, TIMEOUT_MS);
    m_frontleft.configContinuousCurrentLimit(m_maxAmps, TIMEOUT_MS);
    m_frontleft.configPeakCurrentLimit(m_maxAmps, TIMEOUT_MS);

    m_backleft.set(ControlMode.Follower, Robot.m_map.getId(MapKeys.DRIVE_BACKLEFT));
    m_backright.set(ControlMode.Follower, Robot.m_map.getId(MapKeys.DRIVE_BACKRIGHT));
  }

  public void update(double y, double z){

  m_velocity_fps = y * MAXVELOCITY;
  m_velocity_turn_rps = z * (MAX_TURN_RATE_DEG_PER_SEC/RAD_PER_DEG) * m_radius;

  m_rightSpeed = (m_velocity_fps + m_velocity_turn_rps) * REVOLUTIONSPERFOOT * COUNTSPERREVOLUTION);
  m_leftspeed = (m_velocity_fps - m_velocity_turn_rps) * REVOLUTIONSPERFOOT * COUNTSPERREVOLUTION);
  System.out.printf("ForwardJoy: %f TurnJoy: %f", y, z);
}

  @Override
  public void periodic() {
    m_frontleft.set(ControlMode.Velocity, m_leftspeed);
    m_frontright.set(ControlMode.Velocity, m_rightspeed);

    System.out.printf("ForwardVal: %f TurnVal: %f FowardAndTurn: %f Conversion: %f Left: %f Right: %f");

    m_frontleft.set(ControlMode.Velocity, -m_leftspeed);
    m_frontright.set(ControlMode.Velocity, -m_rightspeed);
  }

  @Override
  public void initDefaultCommand() {
  }

public Object getRightEncoder() {
	return null;
}

public Object getLeftEncoder() {
	return null;
}

public void arcadeDrive(double driveForwardSpeed, double driveForwardSpeed2) {
}

public void stop() {
}
}