import java.awt.geom.Point2D;
import java.util.List;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.Color;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import java.util.*;

/**
 * This class uses the light sensor in order to help the robot localise itself.
 * It begins by moving the robot into a position that efficiently reads the lines 
 * of all 4 axes. It then collects the angles at each axis and uses that to correct 
 * the odometer reading (understand where it is on the grid).
 * @author Karine Mellata and Mustafa Khawaja
 *
 */

public class LightLocalizer{
	private Odometer odometer;
	EV3ColorSensor colorSensor;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private Navigation navigator;
	private float[] color;
	SampleProvider us;
	private static final int ROTATE_SPEED = 50;
	private static double WHEEL_RADIUS = 2.1; 
	private static double TRACK = 12.5;
	private int lightIntensity;
	private int lineRead = 0;
	private double[] angles = new double[4];
	private double distToSensor = 16; 
	int blackLine = 0;
	boolean firstBlackLine = true;
	int calib = 0;

	/**
	 * Constructor
	 * 
	 * @param odometer
	 * @param leftMotor
	 * @param rightMotor
	 * @param navigator
	 */
	public LightLocalizer(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Navigation navigator) {
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.navigator = navigator;
		Port sensorPort = LocalEV3.get().getPort("S1");
		colorSensor = new EV3ColorSensor(sensorPort);
		us = colorSensor.getMode("Red");
		this.color = new float[colorSensor.sampleSize()];
		
	}
	/**
	 * This method performs the turning around in order to gather angles and 
	 * moves the robot accordingly.
	 */

		public void localise() {
			//Move into a position that will allow the sensor to cross all 4 axes lines
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, -45), true); //Point to origin
			rightMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, -45), false);
			leftMotor.rotate(convertDistance(WHEEL_RADIUS,5), true); 
			rightMotor.rotate(convertDistance(WHEEL_RADIUS,5), false);
			
			//While loop that will stop when all 4 axes have been crossed
			while (lineRead < 4){
				leftMotor.forward(); //Rotation on itself
				rightMotor.backward(); //Rotation on itself
				us.fetchSample(color, 0);
				lightIntensity = (int) (color[0] * 100);
				if(lightIntensity < 35){ //Line read
					Sound.beep();
					lineRead++;
					angles[lineRead-1] = Math.toDegrees(odometer.getTheta()); //Store the value of the current angle in an array
				}
			}
			leftMotor.stop();
			rightMotor.stop();
			double thetaY = angles[3] - angles[1]; //Angle between negative Y and positive Y
			double thetaX = angles[2] - angles[0]; //Angle between positive X and negative X
			double thetaXRad = Math.toRadians(thetaX);
			double thetaYRad = Math.toRadians(thetaY);
			double xCorrected = (-1)*(distToSensor)*Math.cos(thetaYRad/2); //Correction applied to X 
			double yCorrected = (-1)*(distToSensor)*Math.cos(thetaXRad/2); //Correction applied to Y
			double thetaCorrection =  (90 + (thetaY/2) + 180 - angles[3]); //Angle to which the robot now has to turn in order to be facing the origin
			double thetaCorrectionRad = thetaCorrection*Math.PI*2/360;
			leftMotor.stop();
			rightMotor.stop();

			navigator.turnTo(thetaCorrectionRad); 
			//Update the odometer
			odometer.setPosition(new double [] {xCorrected, yCorrected, thetaCorrectionRad}, new boolean [] {true, true, true});
			Point2D.Double zero = new Point2D.Double(0,0);
			
			//Final position
			navigator.travelTo(zero); 
			navigator.turnTo(0);
		}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}
