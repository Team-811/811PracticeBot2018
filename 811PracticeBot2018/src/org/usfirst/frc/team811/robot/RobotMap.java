package org.usfirst.frc.team811.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.*;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap implements Config 
{
	//objects
	public static Joystick joystick1;
	public static Joystick joystick2;
	
    public static SpeedController drivefrontleft;
    public static SpeedController drivebackleft;
    public static SpeedControllerGroup driveLeft;
	public static SpeedController drivefrontright;
    public static SpeedController drivebackright;
    public static SpeedControllerGroup driveRight;
    public static Encoder driveEncoder;
    public static DifferentialDrive driveTrain;
    public static AnalogGyro driveGyro;
    public static PIDController pid;
    public static AHRS ahrs;
    public static PIDController turnController;
    

    
    public void init() 
    {
    	//initialize
    	joystick1 = new Joystick(1);
        joystick2 = new Joystick(2);
    	
    	
        drivefrontleft = new Talon(FRONT_LEFT_PORT);
        drivebackleft = new Talon(BACK_LEFT_PORT);
        driveLeft = new SpeedControllerGroup(drivefrontleft, drivebackleft);
        drivefrontright = new Talon(FRONT_RIGHT_PORT);
        drivebackright = new Talon(BACK_RIGHT_PORT);
        driveRight = new SpeedControllerGroup(drivefrontright, drivebackright);
        driveTrain = new DifferentialDrive(driveLeft, driveRight);
        driveEncoder = new Encoder(DRIVE_ENCODER_PORT_1, DRIVE_ENCODER_PORT_2);
        driveEncoder.setReverseDirection(false);
        driveEncoder.setDistancePerPulse(DRIVE_DISTANCE_PER_PULSE);
        ahrs = new AHRS(SPI.Port.kMXP);
        
       
    }
}
