
package org.usfirst.frc.team5586.robot;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;
import edu.wpi.first.wpilibj.Timer;


public class Robot extends IterativeRobot {
	final String defaultAuto = "Blank";
	final String customAuto1 = "Vision Shooting";
	final String customAuto2 = "Vision Driving";
	final String customAuto3 = "Move";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<String>();
    Victor mFrontLeft = new Victor(0);
    Victor mBackLeft = new Victor(1);
    Victor mFrontRight = new Victor(2);
    Victor mBackRight = new Victor(3);
    RobotDrive drive = new RobotDrive(mFrontLeft, mBackLeft, mFrontRight, mBackRight);
    Spark intake = new Spark(7);
    Spark winch = new Spark(6);
    Joystick joystick = new Joystick(0);
    ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    private static final int IMG_WIDTH = 320;
	private static final int IMG_HEIGHT = 240;
	Pipeline pipeline;
	VisionThread visionThread;
	private double centerX = 0.0;
	private double centerY = 0.0;
	private final Object imgLock = new Object();
	boolean rotation;
	double timer = 0;
	double moveX = 0;
	double moveY = 0;
	double moveZ = 0;
    
    public void robotInit() {
        gyro.calibrate(); 
        mFrontRight.setInverted(true);
        mBackRight.setInverted(true);
        
        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
        
        visionThread = new VisionThread(camera, new Pipeline(), pipeline -> {
            if (!pipeline.filterContoursOutput().isEmpty()) {
                Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
                synchronized (imgLock) {
                    centerX = r.x + (r.width / 2);
                    centerY = r.y + (r.height / 2);
                }
            }
        });
        visionThread.start();
        chooser.addDefault("Blank Auto", defaultAuto);
		chooser.addObject("Shooting Assist", customAuto1);
		chooser.addObject("Driving Assist", customAuto2);
		chooser.addObject("Move Forward", customAuto3);
		SmartDashboard.putData("Auto choices", chooser);
    }
    
    public void autonomousInit() {
    	autoSelected = chooser.getSelected();
		System.out.println("Auto selected: " + autoSelected);
		System.out.println("Test");
		timer = 0;

    }
    
	public void autonomousPeriodic() {
		
		timer += 0.002;
		SmartDashboard.putNumber("Time in Auto", timer);
		double centerX;
		double centerY;
		switch (autoSelected) {
		case customAuto3:
			
			boolean foundX = false;
			boolean foundY = false;
			double t = 3;
			
			synchronized (imgLock) {
				centerX = this.centerX;
				centerY = this.centerY;
			}
			
			double offX = (centerX - (IMG_WIDTH / 2));
			double offY = (centerY - (IMG_HEIGHT /2));
			
			if(offX < t && offX > -t) {
				foundX = true;
			} else { 
				foundX = false;
				moveX = offX * 0.005;
				moveX += timer * 0.05;
			}
			
			if(offY < t && offY > -t) {
				foundY = true;
			} else { 
				foundY = false;
				moveY = offY * 0.005;
				moveY += timer * 0.05;
			}
			
			
			SmartDashboard.putNumber("Off X", offX);
			SmartDashboard.putNumber("Off Y", offY);
			SmartDashboard.putBoolean("Found X", foundX);
			SmartDashboard.putBoolean("Found Y", foundY);
			
			drive.mecanumDrive_Cartesian(moveX, -moveY, 0, gyro.getAngle());
			
			break;
		case customAuto2:
			synchronized (imgLock) {
				centerX = this.centerX;
			}
			double move = (centerX - (IMG_WIDTH / 2)) * 0.005;
			drive.mecanumDrive_Cartesian(move, 0, 0, gyro.getAngle());
			break;
		case customAuto1:
			synchronized (imgLock) {
				centerX = this.centerX;
			}
			double turn = centerX - (IMG_WIDTH / 2);
			// shooter.set(turn * 0.005);
			break;
		case defaultAuto:
		default:
			drive.mecanumDrive_Cartesian(0, 0, 0, 0);
			break;
		}
	}

    public void teleopPeriodic() {
        double xSpeed;
        double ySpeed;
        double zSpeed;
        boolean toggle1 = true;
        boolean toggle2 = true;
        boolean toggle3 = true;
        boolean toggle4 = true;
        boolean toggle5 = true;
        boolean fullSpeed = false;
        boolean rotation = false;
        boolean robotCentric = true;
        boolean shooterOn = false;
        boolean intakeOn = false;
        
        SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
        SmartDashboard.putBoolean("Rotation", rotation);
        SmartDashboard.putBoolean("Robot Centric", robotCentric);
        
    	if(toggle1 && joystick.getRawButton(8)) {
    		toggle1 = false;
    		robotCentric = !robotCentric;
    	} else if(joystick.getRawButton(8) == false) {
    		toggle1 = true;
    	}
    	
    	/* if(toggle2 && joystick.getRawButton(3)) {
    		toggle2 = false;
    		fullSpeed = !fullSpeed;
    	} else if(joystick.getRawButton(3) == false) {
    		toggle2 = true;
    	} */
    	
    	if(toggle3 && joystick.getRawButton(9)) {
    		toggle3 = false;
    		rotation = !rotation;
    	} else if(joystick.getRawButton(9) == false) {
    		toggle3 = true;
    	}
    	
    	//IO portion, where we get inputs and move actuators
    	
    	if(joystick.getX() < .1 && joystick.getX() > -.1) {
    		xSpeed = 0;
    	} else {
    		xSpeed = joystick.getX();
    	}
    	
    	if(joystick.getY() < .1 && joystick.getY() > -.1) {
    		ySpeed = 0;
    	} else {
    		ySpeed = joystick.getY();
    	}
    	
    	if(joystick.getTwist() < .2 && joystick.getTwist() > -.2) {
    		zSpeed = 0;
    	} else {
    		zSpeed = joystick.getTwist();
    	}
    	
    	if(joystick.getRawButton(2) == true) {
    		//shooter.set(1);
    	} else {
    		//shooter.set(0);
    	}
    	
    	if(joystick.getRawButton(3) == true) {
    		winch.set(1);
    	} else {
    		winch.set(0);
    	}
    	
	    	if(joystick.getRawButton(1) == true) {
	    		intake.set(1);
	    	} else {
    		intake.set(0);
    	}
    	
    	if(joystick.getRawButton(7) == true) {
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
