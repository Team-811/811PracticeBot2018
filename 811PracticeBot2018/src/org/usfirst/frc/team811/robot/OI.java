package org.usfirst.frc.team811.robot;

import org.usfirst.frc.team811.robot.commands.*;
import org.usfirst.frc.team811.robot.RobotMap;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI implements Config 
{
	//creating buttons
	public JoystickButton intake_in;
	public JoystickButton intake_out;
	public JoystickButton intake_stop;
	public JoystickButton shoot;
	//public JoystickButton stopShoot;
	public JoystickButton climber_up;
	public JoystickButton climber_down;
	public JoystickButton gyro_reset;
	public JoystickButton servo_preset;
	public JoystickButton winch_down;
	public JoystickButton shoot_distance;
	
	public OI() 
	{
		//Button initialize
		
		
		gyro_reset = new JoystickButton(RobotMap.joystick1, GYRO_RESET_BUTTON);
		//gyro_reset.whenPressed(new gyro_reset());
		
		//SmartDashboard buttons
		
	}
}