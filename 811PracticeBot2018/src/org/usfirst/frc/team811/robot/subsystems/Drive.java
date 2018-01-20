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
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

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
    Encoder driveEncoderLeft = RobotMap.driveEncoderLeft;
    Encoder driveEncoderRight = RobotMap.driveEncoderRight;
    EncoderFollower leftFollower;
    EncoderFollower rightFollower;
    Trajectory trajectory;
    		
    //AnalogGyro driveGyro = RobotMap.driveGyro;
    AHRS ahrs = RobotMap.ahrs;
    PIDController turnController = RobotMap.turnController;
    
    double max_velocity = 1.7;
    double max_acceleration = 2.0;
    double max_jerk = 60.0;
    double wheel_diameter = 0.206375;
    double wheel_base_distance = 0.3683;
    int encoder_rotation = 1000;
    double kI = 0.0;
    double kP = 1.0;
    double acceleration_gain = 0.3;
    //TODO
    double absolute_max_velocity = 1.7;
    
    
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
    
    /*
    public void driveAuto(double driveDistance) {		//TODO drive distance!
    	double turnVal = ahrs.getAngle();
    	
    	//driveEncoder.setDistancePerPulse(DRIVE_DISTANCE_PER_PULSE);
    	
    	while (driveEncoder.getDistance() <= driveDistance) {
    		driveTrain.tankDrive(-.3, -.4);
    	}
    }
    */
    
    
    public void gyroReset() {
    	ahrs.reset();
    }
    
    
	public void generateTrajectory() {

		

		Waypoint[] points = new Waypoint[] { new Waypoint(-5, -0, 0), // Waypoint @ x=-4, y=-1, exit
																						// angle=-45 degrees
				new Waypoint(-2, 0, 0), // Waypoint @ x=-2, y=-2, exit angle=0 radians
				new Waypoint(0, 0, 0) // Waypoint @ x=0, y=0, exit angle=0 radians
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, 0.05, max_velocity, max_acceleration, max_jerk);
		trajectory = Pathfinder.generate(points, config);
		TankModifier modifier = new TankModifier(trajectory).modify(wheel_base_distance);
		leftFollower = new EncoderFollower(modifier.getLeftTrajectory());
		rightFollower = new EncoderFollower(modifier.getRightTrajectory());
		
		/*
		for (int i = 0; i < trajectory.length(); i++) {
		    Trajectory.Segment seg = trajectory.get(i);
		    
		    System.out.printf("%f,%f,%f,%f,%f,%f,%f,%f\n", 
		        seg.dt, seg.x, seg.y, seg.position, seg.velocity, 
		            seg.acceleration, seg.jerk, seg.heading);
		 
		}
		*/
	}
	public void configureFollower() {
		
		leftFollower.configureEncoder(driveEncoderLeft.getRaw() , encoder_rotation, wheel_diameter);
		rightFollower.configureEncoder(driveEncoderRight.getRaw(), encoder_rotation, wheel_diameter);
		leftFollower.configurePIDVA(kP, 0.0, kP, 1 / absolute_max_velocity, acceleration_gain);
		rightFollower.configurePIDVA(kP, 0.0, kP, 1 / absolute_max_velocity, acceleration_gain);
		
		
		
	}

	public void followTrajectory() {

		double l = leftFollower.calculate(driveEncoderLeft.getRaw());
		double r = rightFollower.calculate(driveEncoderRight.getRaw());

		double gyro_heading = ahrs.getYaw(); // Assuming the gyro is giving a value in degrees
		double desired_heading = Pathfinder.r2d(leftFollower.getHeading()); // Should also be in degrees

		double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
		double turn = 0.8 * (-1.0 / 80.0) * angleDifference;

		driveTrain.tankDrive(-l + turn , -r - turn);
	}	

	 
}

