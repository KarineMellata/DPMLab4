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


public class LightLocalizer{
	private List<Point2D.Double> positions;
	private Odometer odometer;
	EV3ColorSensor colorSensor;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private Navigation navigator;
	private float[] color;
	SampleProvider us;
	private static final int FORWARD_SPEED = 50;
	private static final int ROTATE_SPEED = 50;
	private static double WHEEL_RADIUS = 2.1;
	private static double TRACK = 12.5;
	private static int i = 0 ;
	private int lightIntensity;
	private int lineRead = 0;
	private double[] angles = new double[4];
	private double distToSensor = 15; //TO BE CHANGED
	int blackLine = 0;
	boolean firstBlackLine = true;
	int calib = 0;

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

	public void goToPoint() {
		us.fetchSample(color, 0);
		lightIntensity = (int) (color[0] * 100);
		boolean notCrossed = true;
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, 45.0), true);
		rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, 45.0), false);
		if(lightIntensity < 35) {
			Sound.beep();
			notCrossed = false;
			leftMotor.stop();
			rightMotor.stop();
			leftMotor.rotate(-convertDistance(WHEEL_RADIUS,15), true); // backward
			rightMotor.rotate(-convertDistance(WHEEL_RADIUS,15), false);
			
			leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, 45.0), true);
			rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, 45.0), false);
		}
		else{
			notCrossed = true;
		}
		while(notCrossed) {
			
			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);
			leftMotor.forward();
			rightMotor.forward();
			
		}
	}

		public void localise() {
			
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, -45), true);
			rightMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, -45), false);
			leftMotor.rotate(convertDistance(WHEEL_RADIUS,5), true); // go forward 5 cm slow
			rightMotor.rotate(convertDistance(WHEEL_RADIUS,5), false);
				
			while (lineRead < 4){
				//line read
				leftMotor.forward();
				rightMotor.backward();
				us.fetchSample(color, 0);
				lightIntensity = (int) (color[0] * 100);
				if(lightIntensity < 35){
					Sound.beep();
					lineRead++;
					angles[lineRead-1] = Math.toDegrees(odometer.getTheta());
					
					
				}

				
				//assign each theta reading for each axis, lineRead = 1 when we hit -x axis
			}
			leftMotor.stop();
			rightMotor.stop();
			double thetaY = angles[3] - angles[1];
			double thetaX = angles[2] - angles[0];
			double thetaXRad = Math.toRadians(thetaX);
			double thetaYRad = Math.toRadians(thetaY);
			double xCorrected = (-1)*(distToSensor)*Math.cos(thetaYRad/2);
			double yCorrected = (-1)*(distToSensor)*Math.cos(thetaXRad/2);
			double thetaCorrection =  (90 + (thetaY/2) +180 - angles[3]);// +180
			double thetaCorrectionRad = thetaCorrection*Math.PI*2/360;

			navigator.turnTo(thetaCorrectionRad);

			odometer.setPosition(new double [] {xCorrected, yCorrected, 0}, new boolean [] {true, true, true});
			Point2D.Double zero = new Point2D.Double(0,0);
			navigator.travelTo(zero);
			navigator.turnTo(0);
		}
		
	//correct the angles then assign to odometer (TODO) then turn to corrected theta and start navigation

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}
