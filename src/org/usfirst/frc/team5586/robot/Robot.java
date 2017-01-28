
package org.usfirst.frc.team5586.robot;

import java.io.IOException;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Spark;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
    final String defaultAuto = "Default";
    final String customAuto = "My Auto";
    String autoSelected;
    SendableChooser chooser;
    Victor mFrontLeft = new Victor(0);
    Victor mBackLeft = new Victor(1);
    Victor mFrontRight = new Victor(2);
    Victor mBackRight = new Victor(3);
    RobotDrive drive = new RobotDrive(mFrontLeft, mBackLeft, mFrontRight, mBackRight);
    Spark shooter = new Spark(4);
    Joystick joystick = new Joystick(0);
    ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    double xSpeed;
    double ySpeed;
    double zSpeed;
    boolean toggle1 = true;
    boolean toggle2 = true;
    boolean toggle3 = true;
    boolean toggle4 = true;
    boolean fullSpeed = false;
    boolean rotation = false;
    boolean robotCentric = false;
    boolean shooterOn = false;
    CameraServer server;
    private final NetworkTable grip = NetworkTable.getTable("grip");
    
    public void robotInit() {
        chooser = new SendableChooser();
        chooser.addDefault("Default Auto", defaultAuto);
        chooser.addObject("My Auto", customAuto);
        SmartDashboard.putData("Auto choices", chooser);
        gyro.calibrate();
        mFrontLeft.setInverted(true);
        mBackLeft.setInverted(true);
        
        
        server = CameraServer.getInstance();
        server.setQuality(50);
        server.startAutomaticCapture("cam0");
        try {
        	new ProcessBuilder("/home/lvuser/grip").inheritIO().start();
        } catch (IOException e) {
        	e.printStackTrace();
        }
        
        
    }
    
    public void autonomousInit() {
    	autoSelected = (String) chooser.getSelected();
		System.out.println("Auto selected: " + autoSelected);
    }
    
    public void autonomousPeriodic() {
    	switch(autoSelected) {
    	case customAuto:
        //Put custom auto code here   
            break;
    	case defaultAuto:
    	default:
    	//Put default auto code here
            break;
    	}
    }

    public void teleopPeriodic() {
        SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
        
        for (double area : grip.getNumberArray("targets/area", new double[0])){
			System.out.println("Found contour with area: " + area);
		}
    	
    	// Toggle selection portion of the teleop period
    	
    	if(toggle1 && joystick.getRawButton(8)) {
    		toggle1 = false;
    		robotCentric = !robotCentric;
    	} else if(joystick.getRawButton(8) == false) {
    		toggle1 = true;
    	}
    	
    	if(toggle2 && joystick.getRawButton(3)) {
    		toggle2 = false;
    		fullSpeed = !fullSpeed;
    	} else if(joystick.getRawButton(3) == false) {
    		toggle2 = true;
    	}
    	
    	if(toggle3 && joystick.getRawButton(2)) {
    		toggle3 = false;
    		rotation = !rotation;
    	} else if(joystick.getRawButton(2) == false) {
    		toggle3 = true;
    	}
    	
    	if(toggle4 && joystick.getRawButton(1)) {
    		toggle4 = false;
    		shooterOn = !shooterOn;
    	} else if(joystick.getRawButton(1) == false) {
    		toggle4 = true;
    	}
    	
    	//IO portion, where we get inputs and move actuators
    	if(joystick.getX() < .1 && joystick.getX() > -.1) {
    		xSpeed = 0;
    	} else {
    		if(fullSpeed) {
    			xSpeed = joystick.getX();
    		} else {
    			xSpeed = joystick.getX() * .75;
    		}
    	}
    	
    	if(joystick.getY() < .1 && joystick.getY() > -.1) {
    		ySpeed = 0;
    	} else {
    		if(fullSpeed) {
    			ySpeed = joystick.getY();
    		} else {
    			ySpeed = joystick.getY() * .75;
    		}
    	}
    	
    	if(rotation) {
        	zSpeed = joystick.getTwist() * .75;
        } else {
        	zSpeed = 0;
        }
    	
    	if(joystick.getRawButton(1) == true) {
    		shooter.set(.80);
    	} else {
    		shooter.set(0);
    	}
    	
    	if(joystick.getRawButton(5) == true) {
    		gyro.reset();
    	}
    	
        if(robotCentric) {
        	drive.mecanumDrive_Cartesian(xSpeed, ySpeed, zSpeed, 0);
        } else {
        	drive.mecanumDrive_Cartesian(xSpeed, ySpeed, zSpeed, gyro.getAngle());
        }
    }

    public void testPeriodic() {
    
    }
    
}
