import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class is the first method used by the robot to determine the initial orientation of the robot
 * and reposition the robot parallel to the Y axis.
 * @author Karine Mellata and Mustafa Khawaja
 *
 */

public class UltrasonicLocalizer extends Thread implements UltrasonicController{
	public enum LocalizationType {F_E,R_E};
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private static final int ROTATE_SPEED = 50;
	private static final int FILTER_OUT = 3;
	private static int tunedDistance = 35;
	private static int bufferDistance = 2;
	private static double WHEEL_RADIUS = 2.1;
	private static double TRACK = 12.5;
	private static int acceleration = 3000;
	private int distance;
	private int filterControl;
	private int lastDistance;
	LocalisationLab.LocalizationType type;
	
	/**
	 * Constructor
	 * @param odometer
	 * @param leftMotor
	 * @param rightMotor
	 * @param navigator
	 * @param type
	 */

	public UltrasonicLocalizer(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Navigation navigator, LocalisationLab.LocalizationType type) {
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.type = type;
		filterControl = 0;
	}

	public void run() {
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(acceleration);
		}
		// wait 5 seconds
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// there is nothing to be done here because it is not expected that
			// the odometer will be interrupted by another thread
		}

		localise();
	}

	/**
	 * This method will determine the average between the angle at the two wall
	 * This is an approximate way to determine the corner of the two walls. 
	 * It will then reposition itself at an angle of zero according to the values read
	 */
	public void localise() {
		int correctionAngleRE = 38; //These values are experimental values
		int correctionAngleFE = 15; //and were determined by tuning and testing
		double angle1,angle2;
		double avgTheta;
		double zeroTheta;
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		if(type == LocalisationLab.LocalizationType.F_E) {
			while(distance < tunedDistance - bufferDistance) {
				leftMotor.forward();
				rightMotor.backward();
			}
			while(distance > tunedDistance) {
				leftMotor.forward();
				rightMotor.backward();
			}
			Sound.beep(); //We saw the first wall
			angle1 = Math.toDegrees(odometer.getTheta());
			while(distance < tunedDistance - bufferDistance) {
				leftMotor.backward();
				rightMotor.forward();
			}
			while(distance > tunedDistance) {
				leftMotor.backward();
				rightMotor.forward();
			}
			Sound.beep(); //We saw the second wall
			leftMotor.stop();
			rightMotor.stop();
			angle2 = Math.toDegrees(odometer.getTheta());

			if(angle1 > angle2){ 
				angle1 = angle1 - 360;
			}
			//Average angle of the corner
			avgTheta = (angle1 + angle2)/2;
			zeroTheta =  angle2 - avgTheta - correctionAngleFE;
			//Rotate to calculated zero angle
 			leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, zeroTheta), true);
 			rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, zeroTheta), false);
 			//Update the odometer
 			double[] position = new double [] {0.0, 0.0, 0};
 			boolean[] correct = new boolean [] {true, true, true};
 			odometer.setPosition(position, correct);
		}
		else {
			//Same logic as falling edge, except we are turning the other way around
			while(distance < tunedDistance - bufferDistance) {
				leftMotor.backward();
				rightMotor.forward();
			}
			while(distance > tunedDistance) {
				leftMotor.backward();
				rightMotor.forward();
			}
			Sound.beep(); //We saw the first wall
			angle1 = Math.toDegrees(odometer.getTheta());
			while(distance < tunedDistance - bufferDistance) {
				leftMotor.forward();
				rightMotor.backward();
			}
			while(distance > tunedDistance) {
				leftMotor.forward();
				rightMotor.backward();
			}
			Sound.beep(); //We saw the second wall
			leftMotor.stop();
			rightMotor.stop();
			angle2 = Math.toDegrees(odometer.getTheta());
			//Same logic as falling edge
			if(angle1 > angle2){
				angle1 = angle1 - 360;
			}
			avgTheta = (angle1 + angle2)/2;
			zeroTheta =  angle2 - avgTheta + correctionAngleRE; //Only change is adding the angle instead
 			leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, zeroTheta), true);
 			rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, zeroTheta), false);
 			double[] position = new double [] {0.0, 0.0, 0};
 			boolean[] correct = new boolean [] {true, true, true};
 			odometer.setPosition(position, correct);
		}
	}

	@Override
	/**
	 * This method will process the distance and apply a filter to it to filter out extreme values
	 */
	public void processUSData(int distance) {
		int result = 0;
		int distanceBound = 255; //Bound for high values
		if (distance > distanceBound && filterControl < FILTER_OUT) { 
			//We wait for 3 high values before accepting it
			filterControl ++;
			result = lastDistance;
		} else if (distance > distanceBound){
			result = distanceBound; 
		} else {
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