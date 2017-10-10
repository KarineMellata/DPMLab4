import java.awt.geom.Point2D;
import java.util.List;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class UltrasonicLocalizer extends Thread implements UltrasonicController{
	public enum LocalizationType {F_E,R_E};
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private Navigation navigator;
	private SampleProvider us;
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 50;
	private static final int FILTER_OUT = 3;
	private static int WALL_DIST = 35;
	private static int WALL_GAP = 2;
	private static double WHEEL_RADIUS = 2.1;
	private static double TRACK = 12.5;
	private static int i = 0 ;
	private int distance;
	private int filterControl;
	private int lastDistance;
	LocalisationLab.LocalizationType type;

	public UltrasonicLocalizer(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Navigation navigator, LocalisationLab.LocalizationType type) {
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.navigator = navigator;
		this.us = us;
		this.type = type;
		filterControl = 0;
	}

	public void run() {
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(3000);
		}
		// wait 5 seconds
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// there is nothing to be done here because it is not expected that
			// the odometer will be interrupted by another thread
		}

		doLocalisation();
	}

	public void doLocalisation() {
		double [] position = new double[3];
		double angle1,angle2;
		double avgTheta;
		double zeroTheta;
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		if(type == LocalisationLab.LocalizationType.F_E) {
			while(distance < WALL_DIST - WALL_GAP) {
				leftMotor.forward();
				rightMotor.backward();
			}
			while(distance > WALL_DIST) {
				leftMotor.forward();
				rightMotor.backward();
			}
			Sound.beep(); //We saw the wall!
			angle1 = Math.toDegrees(odometer.getTheta());
			while(distance < WALL_DIST - WALL_GAP) {
				leftMotor.backward();
				rightMotor.forward();
			}
			while(distance > WALL_DIST) {
				leftMotor.backward();
				rightMotor.forward();
			}
			Sound.beep(); //We saw the wall!
			leftMotor.stop(true);
			rightMotor.stop(true);
			angle2 = Math.toDegrees(odometer.getTheta());

			if(angle1 > angle2){
				angle1 = angle1 - 360;
			}
			avgTheta = (angle1 + angle2)/2;
			zeroTheta =  angle2 - avgTheta - 5;//or width 10.5 and lower theta to 30?
 			leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, zeroTheta), true);
 			rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, zeroTheta), false);
 			odometer.setPosition(new double [] {0.0, 0.0, 0}, new boolean [] {true, true, true});
		}
		else {
			while(distance < WALL_DIST - WALL_GAP) {
				leftMotor.backward();
				rightMotor.forward();
			}
			while(distance > WALL_DIST) {
				leftMotor.backward();
				rightMotor.forward();
			}
			Sound.beep(); //We saw the wall!
			angle1 = Math.toDegrees(odometer.getTheta());
			while(distance < WALL_DIST - WALL_GAP) {
				leftMotor.forward();
				rightMotor.backward();
			}
			while(distance > WALL_DIST) {
				leftMotor.forward();
				rightMotor.backward();
			}
			Sound.beep(); //We saw the wall!
			leftMotor.stop(true);
			rightMotor.stop(true);
			angle2 = Math.toDegrees(odometer.getTheta());

			if(angle1 > angle2){
				angle1 = angle1 - 360;
			}
			avgTheta = (angle1 + angle2)/2;
			zeroTheta =  angle2 - avgTheta - 5;//or width 10.5 and lower theta to 30?
 			leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, zeroTheta), true);
 			rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, zeroTheta), false);
 			odometer.setPosition(new double [] {0.0, 0.0, 0}, new boolean [] {true, true, true});
		}
	}

	@Override
	public void processUSData(int distance) {
		int result = 0;
		if (distance > 50 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the filter value
			filterControl ++;
			result = lastDistance;
		} else if (distance > 50){
			// true 255, therefore set distance to 255
			result = 50; //clips it at 50
		} else {
			// distance went below 255, therefore reset everything.
			filterControl = 0;
			result = distance;
		}
		this.distance = result;
	}

	@Override
	public int readUSDistance() {
		// TODO Auto-generated method stub
		return 0;
	}
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}