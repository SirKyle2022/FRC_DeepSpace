package frc.robot;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.PneumaticArmExtend;
import frc.robot.commands.PneumaticArmRetract;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

import frc.robot.commands.PneumaticArmExtend;
import frc.robot.commands.PneumaticArmOff;
import frc.robot.commands.PneumaticArmRetract;


public class OI{


  Joystick joy = new Joystick(0);
  
  private final int CAMERA_BUTTON = 1;
  private final int Lvl2DESC_BUTTON = 2;
  private final int Lvl2CLIMB_BUTTON = 3;
  private final int Lvl3CLIMB_BUTTON = 4;
  public JoystickButton xButton1 = new JoystickButton(joy, CAMERA_BUTTON);
	public JoystickButton backButton1 = new JoystickButton(joy, Lvl2DESC_BUTTON);
	public JoystickButton startButton1 = new JoystickButton(joy, Lvl2CLIMB_BUTTON);
  public JoystickButton playButton1 = new JoystickButton(joy, Lvl3CLIMB_BUTTON);
    

  Joystick joy2 = new Joystick(1);


  private final int HIGHHATCH_BUTTON = 1;
  private final int MIDHATCH_BUTTON = 2;
  private final int LOWHATCH_BUTTON = 3;
  private final int CLAMP_BUTTON = 4;

  public JoystickButton yButton2 = new JoystickButton(joy2, HIGHHATCH_BUTTON);
  public JoystickButton xButton2 = new JoystickButton(joy2, MIDHATCH_BUTTON);
  public JoystickButton aButton2 = new JoystickButton(joy2, LOWHATCH_BUTTON);
  public JoystickButton bButton2 = new JoystickButton(joy2, CLAMP_BUTTON);



  public OI () {

  /**  xButton1.toggleWhenPressed(new CameraToggle);

    rightTrigger2.togglewhenActive(new SlideRightToggle);
    rightTrigger2.whenInactive(new SlideLeftStop);

    backButton1.whenPressed(new Lvl2Descent);
    startButton1.whenPressed(new Lvl2Climb);
    playButton1.whenPressed(new Lvl3Climb);


    yButton2.toggleWhenPressed(new HighHatchInitiate);
    xButton2.toggleWhenPressed(new MidHatchInitiate);
    aButton2.toggleWhenPressed(new LowHatchInitiate);
    */
    bButton2.whenPressed(new PneumaticArmExtend());
    bButton2.whenReleased(new PneumaticArmRetract());

    
  }
 



  public double getForwardValue() {
    return joy.getRawAxis(1);

  }
  public double getTurnValue() {
    return joy.getRawAxis(4);
  }


  public double liftUpValue() {
    return joy2.getRawAxis(0);
  }

  public double liftDownValue() {
    //need to check axes
    return joy2.getRawAxis(1);
  }


  public double getLeftSlideValue() {
    return joy2.getRawAxis(2);
  }

  public double getRightSlideValue() {
    return joy2.getRawAxis(3);
  }


  


}
