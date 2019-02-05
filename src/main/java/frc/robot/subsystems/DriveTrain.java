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
import edu.wpi.first.wpilibj.RobotDrive;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Joystick.AxisType;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // create CAN Talon SRX objects
  private WPI_TalonSRX m_frontleft;
  private WPI_TalonSRX m_backleft;
  private WPI_TalonSRX m_frontright;
  private WPI_TalonSRX m_backright;
  private static final int TIMEOUT_MS = 1000;
  //feet per second velocity
  private static double m_vel_fps;
  private static double m_vel_encps;
  private static final double COUNTSPERFOOT = 1800;
  private static final double MAXVELOCITY = 5;



  public DriveTrain() {
    //initialize + set objects created above
    m_frontleft = new WPI_TalonSRX(RobotMap.frontLeftMotorCanId());
    m_backleft = new WPI_TalonSRX(RobotMap.backLeftMotorCanId());
    m_frontright = new WPI_TalonSRX(RobotMap.frontRightMotorCanId());
    m_backright = new WPI_TalonSRX(RobotMap.backRightMotorCanId());
    configTalons();
    m_vel_fps = 0;
    m_vel_encps = 0;
    //m_left.setInverted(true); invert left side
    //Configure the RobotDrive to reflect the fact that all our motors are
    //Wired backwards and our drivers sensitivity preferences.
    }

  public void update(double y, double z){
  }

  public void configTalons() {
    /* first choose the sensor */
    m_frontright.selectProfileSlot(0, 0);
    m_frontright.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    m_frontright.setSensorPhase(false);

    m_frontleft.selectProfileSlot(0, 0);
    m_frontleft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    m_frontleft.setSensorPhase(false);
    //_talon.configEncoderCodesPerRev(XXX), // if using FeedbackDevice.QuadEncoder
    //_talon.configPotentiometerTurns(XXX), // if using FeedbackDevice.AnalogEncoder or AnalogPot

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

    m_backleft.set(ControlMode.Follower, RobotMap.backLeftMotorCanId());
    m_backright.set(ControlMode.Follower, RobotMap.backRightMotorCanId());
  }

  @Override
  public void initDefaultCommand() {
  }
}