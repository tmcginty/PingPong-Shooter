/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1711.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.networktables.*;
import org.opencv.core.*;
import org.opencv.imgproc.*;

/**
 * This sample program shows how to control a motor using a joystick. In the
 * operator control part of the program, the joystick is read and the value is
 * written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and speed controller inputs also
 * range from -1 to 1 making it easy to work together.
 */
public class Robot extends IterativeRobot {
	// Roborio port connections
	private static final int kJoystickPort = 0;
	private static final int kShooterSparkPort = 0;
	private static final int kLEDRingSparkPort = 1;
	private static final int kHorizontalServoPort = 2;
	private static final int kVerticalServoPort = 3;
	private static final int kTriggerServoPort = 4;
	
	// control system declarations
	private Joystick m_joystick;
	private SpeedController m_mcShooter;
	private SpeedController m_mcLED;
	private Servo m_servoHorizontal;
	private Servo m_servoVertical;
	private Servo m_servoTrigger;
	
	// limits
	private static final int  kCenterHorizontalAngle = 90;
	private static final int  kMinHorizontalAngle = -45;
	private static final int  kMaxHorizontalAngle = 45;

	private static final int  kMaxVerticalAngle = 70;
	private static final int  kMinVerticalAngle = 8;
	
	private static final int  kMinTriggerAngle = 0;
	private static final int  kBlockTriggerAngle = 30;
	private static final int  kMaxTriggerAngle = 50;

	private static final int  kMaxLEDIdleBrightness = 60;
	
	// imaging
	private static final int kMJPEGPort = 1180;
	private static final int kCameraDevID = 0;

	// variables
	private boolean m_bShooterRunning = false;
	NetworkTable m_ntGRIP=NetworkTable.getTable("GRIP/myContoursReport");
	CvSink m_imageSink = new CvSink("CV Image Grabber");
    ImageTracker m_imaging=new ImageTracker();
    boolean m_modeHunt=true;
	boolean m_shootPressed=false;
    
	@Override
	public void robotInit() {

		// define the control system objects
		m_mcShooter = new Spark(kShooterSparkPort);
		m_mcLED = new Spark(kLEDRingSparkPort);
		m_joystick = new Joystick(kJoystickPort);
		m_servoHorizontal = new Servo(kHorizontalServoPort);
		m_servoVertical = new Servo(kVerticalServoPort);
		m_servoTrigger = new Servo(kTriggerServoPort);
	    	    
	    // initialize Servo positions
	    m_servoTrigger.setAngle(kBlockTriggerAngle);
	    m_servoHorizontal.setAngle(kCenterHorizontalAngle); 
	    m_servoVertical.setAngle(kMinVerticalAngle); 
	}

	
	@Override
	public void  teleopInit() {
	    // flash the LED to state idle
//	    setLEDIdle();
		m_mcLED.set(0.2);
	    analyzeImage();
		
	    // start looking for a target
//	    huntForTarget();
//	    headUp(kMaxVerticalAngle);
	}
	
	@Override
	public void teleopPeriodic() {
		if(m_joystick.getRawButton(3))
			m_mcShooter.set(0.22);
//		else
//			m_mcShooter.set(0);

		if(m_joystick.getRawButton(1)&&m_shootPressed==false) {
			m_shootPressed=true;
			shoot();
		}
		else {
			m_shootPressed=false;
		}
	}
	
	/**
	 * Turn the shooter to an angle relative to current position. This operation
	 * respects the min and max turn limits. Movement is from the current position to new angle
	 * 
	 * @param angle angle +/- to move to from current position.
	 * @return true if head can move or false if limit exceeded
	 */
	private boolean turnHead(int angle) {
		// obtain the current Horizontal servo position
		double newAngle=m_servoHorizontal.getAngle()+angle;
		
		// ensure angle is in range to turn
		if(newAngle>=(kMinHorizontalAngle+kCenterHorizontalAngle)&&newAngle<=(kMaxHorizontalAngle+kCenterHorizontalAngle)) {
			m_servoHorizontal.setAngle(newAngle);
			return true;
		}
		// return limit exceeded
		return false;
	}

	/***
	 * Move the shooter head up to the requested position
	 * @param angle	angle to move to from 0 position
	 */	
	private void headUp(int angle) {
		// ensure angle is in range to turn
		if(angle>=kMinVerticalAngle&&angle<=kMaxVerticalAngle) {
			m_servoVertical.setAngle(angle);
		}		
	}
	
	/***
	 * Move the shooter head down to the max position
	 * @param angle	angle to move to from 0 position
	 */	
	private void headDown(int angle) {
		// obtain the current vertical servo position
		double currentAngle=m_servoVertical.getAngle();
		
		// ensure angle is in range to turn
		if(angle<=kMaxVerticalAngle) {
			m_servoVertical.setAngle(angle);
		}		
	}
	
	/***
	 * Actuate the shooter mechanism
	 */
	private void shoot() {
		// ensure the shooter is running
		m_bShooterRunning=true;
		// move the trigger in  controlled manner to not fire the 
		// ball out of the slot
		(new Thread() { public void run() {
			boolean bComplete=false;
			boolean bIsFiring=true;
			// start the wheel
			m_mcShooter.set(0.22);
			try {
				sleep(2000);			// delay between ramp moves
			} catch(InterruptedException e) {
			}

			// use a thread to independently perform the shooting process
			while(m_bShooterRunning&&bComplete==false) {
				try {
					if(bIsFiring) {
						// push ball up over the endstop
						if(m_servoTrigger.getAngle()<kMaxTriggerAngle)
							m_servoTrigger.setAngle(m_servoTrigger.getAngle()+1);
						else
							bIsFiring=false;
					}
					else {
						// retract the trigger
						m_servoTrigger.setAngle(kMinTriggerAngle);
						// next ball time to get into position
						sleep(750);
						// block the next ball
						m_servoTrigger.setAngle(kBlockTriggerAngle);
						bComplete=true;
						sleep(2000);
						m_mcShooter.set(0);
					}
					sleep(100);			// delay between ramp moves
				} catch(InterruptedException e) {
					bComplete=true;
				}
			}
		}}).start(); 
	}
		
	/***
	 * ramp up and down the LED
	 */
	private void setLEDIdle() {
		(new Thread() { public void run() {
			int counter=0;
			boolean rampUp=true;

			while(isEnabled()&&m_modeHunt==true) {
				if(rampUp==true) {
					if(counter<=kMaxLEDIdleBrightness)
						counter++;
					else
						rampUp=false;
				}
				
				if(rampUp==false) {
					if(counter>15)
						counter--;
					else
						rampUp=true;
				}
			
				m_mcLED.set(((float)counter)/100);
				
				try {
					sleep(25);			// delay between ramp 
				} catch(InterruptedException e) {
				}
			}
		}}).start(); 
	}

	/***
	 * move from side to side looking until a target is acquired
	 */
	private void huntForTarget() {
		(new Thread() { public void run() {
			boolean panRight=true;
			int panAngle=0;

			// perform operation only while machine is enabled
			while(isEnabled()) {
				// if we are not in hunt mode then sleep
				if(m_modeHunt==false)
					try {
						sleep(100);
					} catch(InterruptedException ex) {}
				else {
					// no target found, 
					// pan to the right
					if(panRight==true) {
						if(turnHead(1)==false)
							panRight=false;
					}
					
					// pan to the left 
					if(panRight==false) {
						if(turnHead(-1)==false)
							panRight=true;
					}
					System.out.println(m_servoHorizontal.getAngle()+kCenterHorizontalAngle);
				    
				    try {
						// give servo time to move to position
						sleep(100);
					} catch(InterruptedException ex) {}
				}
			}
		}}).start(); 
	}

	
	private void analyzeImage() {
		(new Thread() { public void run() {
			// create and assign the image source (USB Camera)
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		    camera.setResolution(320,240);

		    CvSink cvSink=CameraServer.getInstance().getVideo();
			CvSource outputStream = CameraServer.getInstance().putVideo("CV Output", 320, 240);
			
			Mat m_inputImage=new Mat();
			// perform operation only while machine is enabled
			while(isEnabled()) {
				// grab the image and process
				long frameTime = cvSink.grabFrame(m_inputImage);
		    	Mat output=new Mat();
		    	m_inputImage.copyTo(output);
			    if(frameTime>0) {
			    	m_imaging.process(m_inputImage);
			    	if(m_imaging.findContoursOutput().size()>0) {
			    		// System.out.println(m_imaging.findContoursOutput().size() + " contours(s) detected");
			    		for(int i=0;i<m_imaging.findContoursOutput().size();i++) {
			    			
			    			Rect r=Imgproc.boundingRect(m_imaging.findContoursOutput().get(i));
			    			
			    			// determine if the aspect ratio (w/h) is within the range of our target
			    			// and eliminate anything that is not
			    			double ar=(double)r.width / (double)r.height;
			
							//System.out.println("Aspect ratio: " + ar);
			    			if(ar>=1.6&&ar<=1.8) {
				    			Point ptl=new Point(r.x, r.y);
				    			Point pbr=new Point(r.x+r.width, r.y+r.height);
				    			Scalar s=new Scalar(0,0,255);			// in BGR format
				    			Imgproc.rectangle(output,ptl, pbr, s, 1);
				    			
			    			}
			    		}
			    	}
			    	// display image on channel
			    	outputStream.putFrame(output);
			    	output.release();
			    }
			}
		}}).start(); 
	}

	/***
	 * 
	 * @param height	height
	 * @param distance
	 */
	private void setHeight(int height, int distance) {
	}
}

