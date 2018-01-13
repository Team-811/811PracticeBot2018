package org.usfirst.frc.team811.robot.subsystems;


import java.awt.Robot;

import org.usfirst.frc.team811.robot.RobotMap;
import org.usfirst.frc.team811.robot.commands.drive_w_joysticks;
import org.usfirst.frc.team811.robot.Config;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */

public class Drive extends Subsystem implements Config {
	
    Joystick joy1 = RobotMap.joystick1;
    SpeedController frontleft = RobotMap.drivefrontleft;
    SpeedController backleft = RobotMap.drivebackleft;
    SpeedControllerGroup left = RobotMap.driveLeft;
    SpeedController frontright = RobotMap.drivefrontright;
    SpeedController backright = RobotMap.drivebackright;
    SpeedControllerGroup right = RobotMap.driveRight;
    DifferentialDrive driveTrain = RobotMap.driveTrain;
    Encoder driveEncoder = RobotMap.driveEncoder;
    //AnalogGyro driveGyro = RobotMap.driveGyro;
    AHRS ahrs = RobotMap.ahrs;
    PIDController turnController = RobotMap.turnController;
    
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void driveWithJoy() {
    	
    	double moveVal;
    	double turnVal;
    	
    	if ((joy1.getRawAxis(FORWARD_DRIVE_AXIS) < .2) && (joy1.getRawAxis(FORWARD_DRIVE_AXIS) > -.2)) { 
    		moveVal = 0;
    	} else {
    		moveVal = -joy1.getRawAxis(FORWARD_DRIVE_AXIS);
    	}
    	
    	//if ((joy1.getRawAxis(TURN_DRIVE_AXIS) < .2) && (joy1.getRawAxis(TURN_DRIVE_AXIS) > -.2)) { 
    		//ahrs.reset();
    		//turnVal = ahrs.getYaw() * -.1;
    	//} else {
    		turnVal = -joy1.getRawAxis(TURN_DRIVE_AXIS);
    	//}
    	
    	//driveTrain.arcadeDrive(-1 * moveVal * SPEED_SCALE, turnVal * SPEED_SCALE);
    	driveTrain.arcadeDrive(-1 * moveVal * SPEED_SCALE, turnVal * SPEED_SCALE);
    	
    	/* double leftVal = joy1.getRawAxis(FORWARD_DRIVE_AXIS); in case Joe wants tankdrive
    	 * double rightVal = joy1.getRawAxis(TURN_DRIVE_AXIS);
    	 * driveRobotDrive41.tankDrive(leftVal, rightVal);
    	 */
    }
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new drive_w_joysticks());
    }
    
    public void driveAuto(double driveDistance) {		//TODO drive distance!
    	double turnVal = ahrs.getAngle();
    	
    	//driveEncoder.setDistancePerPulse(DRIVE_DISTANCE_PER_PULSE);
    	
    	while (driveEncoder.getDistance() <= driveDistance) {
    		driveTrain.tankDrive(-.3, -.4);
    	}
    }
    
    
    
    public void gyroReset() {
    	ahrs.reset();
    }
	

	
}

