import lejos.hardware.motor.EV3LargeRegulatedMotor;
import java.awt.geom.*;
import java.util.*;

/**
 * This class contains the method to travel to a point (travelTo) and 
 * the method to turn to the smallest angle possible (turnTo)
 * @author Karine Mellata and Mustafa Khawaja
 *
 */

public class Navigation{
	private List<Point2D.Double> positions;
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private static final int FORWARD_SPEED = 50;
	private static final int ROTATE_SPEED = 50;
	private static double WHEEL_RADIUS = 2.1;
	private static double TRACK = 12.5;
	private static int i = 0 ;
	
	/**
	 * Constructor
	 * 
	 * @param odometer
	 * @param leftMotor
	 * @param rightMotor
	 * @param wheelRadius
	 * @param track
	 */
	
	public Navigation(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double wheelRadius, double track) {
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	}
	
	/**
	 * This method makes to robot turn to a certain point and then travel to it
	 * @param point2d
	 */
	
	public void travelTo(Point2D.Double point2d){ 
		//initialization - where am I ?
		double X_current = odometer.getX();
		double Y_current = odometer.getY();
		Point2D.Double currentPosition = new Point2D.Double(X_current, Y_current);
		//parameters - where am I going ?
		double X = point2d.getX();
		double Y = point2d.getY();
		double sideX = X - X_current;
		double sideY = Y - Y_current;
		double distance = currentPosition.distance(point2d);
		
		double alpha = Math.atan2(sideX, sideY); 
			turnTo(alpha);
			goStraight(distance);

	}
	
	/**
	 * This method processes the radian angle and proceeds with the turn
	 * @param alpha
	 */
	public void turnTo(double alpha){ 	
		double currentAngle = odometer.getTheta();
		if(alpha < 0){  //Shifting the angle to 0 to 2pi instead of -pi to pi
			alpha = ((2.0*Math.PI) + alpha);
		}
		double theta = alpha - currentAngle;
		theta = (theta*360)/(2*Math.PI); //To degrees
		if((theta >= -180) && (theta <= 180)){ //If already smallest angle
			if(theta < 0){ //Turn the other way around by -theta
				leftMotor.setSpeed(ROTATE_SPEED);
			    rightMotor.setSpeed(ROTATE_SPEED);
				leftMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, -theta), true);
				rightMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, -theta), false);
			}
			else{ //Turn normally
				leftMotor.setSpeed(ROTATE_SPEED);
			    rightMotor.setSpeed(ROTATE_SPEED);
				rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK,  theta), true);
				leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK,  theta), false);
			}
		}
		else if(theta < -180){ //Get smallest angle
			theta = theta + 360;
			leftMotor.setSpeed(ROTATE_SPEED);
		    rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK,  theta), true);
			rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK,  theta), false);
		}
		else if(theta > 180){ //Get smallest angle
			theta = 360 - theta;
			leftMotor.setSpeed(ROTATE_SPEED);
		    rightMotor.setSpeed(ROTATE_SPEED);
			rightMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK,  theta), true);
			leftMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK,  theta), false);
		}
	}
	
	/**
	 * This method makes the robot go straight for a distance
	 * @param distance
	 */
	
	public void goStraight(double distance) {
	    leftMotor.setSpeed(FORWARD_SPEED);
	    rightMotor.setSpeed(FORWARD_SPEED);
	    leftMotor.rotate(convertDistance(WHEEL_RADIUS, distance), true);
	    rightMotor.rotate(convertDistance(WHEEL_RADIUS, distance), false);
	}
	  
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
