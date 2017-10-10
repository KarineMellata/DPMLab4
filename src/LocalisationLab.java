
import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;
import java.util.Arrays;
import java.util.List;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

public class LocalisationLab {
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3UltrasonicSensor uSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S4"));
	public static final double WHEEL_RADIUS = 2.1;
	public static final double TRACK = 12.5;
	public static final double TM = 30.48;
	public static Point2D.Double targetPosition;

	static Point2D.Double wp1 = new Point2D.Double(0*TM,1*TM);
	static Point2D.Double wp2 = new Point2D.Double(1*TM,2*TM);
	static Point2D.Double wp3 = new Point2D.Double(1*TM,0*TM);
	static Point2D.Double wp4 = new Point2D.Double(2*TM,1*TM);
	static Point2D.Double wp5 = new Point2D.Double(2*TM,2*TM);
	public enum LocalizationType {F_E, R_E};

	final static List<Point2D.Double> positions = Arrays.asList(wp1,wp2,wp3,wp4,wp5);

	public static void main(String[] args) {
		//Setting up objects
		int buttonChoice;
		final TextLCD t = LocalEV3.get().getTextLCD();
		Odometer odometer = new Odometer(leftMotor, rightMotor);
		Navigation navigation = new Navigation(odometer, leftMotor, rightMotor, WHEEL_RADIUS, TRACK, positions);
		@SuppressWarnings("resource")
		SensorModes us = uSensor;
		SampleProvider usDistance = us.getMode("Distance");	// usDistance provides samples from this instance of the sensor
		float[] usData = new float[usDistance.sampleSize()];		// usData is the buffer in which data are returned
		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, t);
		do {
			// clear the display
			t.clear();

			// ask the user whether the motors should drive in a square or float
			t.drawString("                ", 0, 0);
			t.drawString("  LEFT for F_E  ", 0, 1);
			t.drawString("                ", 0, 2);
			t.drawString(" RIGHT FOR R_E  ", 0, 3);
			t.drawString("                ", 0, 4);

			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {
			UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(odometer, leftMotor, rightMotor, navigation, LocalisationLab.LocalizationType.F_E);
			UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, usLocalizer);
			usPoller.start();
			odometer.start();
			usLocalizer.start();
			odometryDisplay.start();
			
			Button.waitForAnyPress();
			
			LightLocalizer lightLocalizer = new LightLocalizer(odometer,leftMotor, rightMotor,navigation);
			//lightLocalizer.goToPoint();
			lightLocalizer.localise();

		}
		else if (buttonChoice == Button.ID_RIGHT) {
			UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(odometer, leftMotor, rightMotor, navigation, LocalisationLab.LocalizationType.R_E);
			UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, usLocalizer);
			usPoller.start();
			odometer.start();
			usLocalizer.start();
			odometryDisplay.start();
			
			Button.waitForAnyPress();
			
			LightLocalizer lightLocalizer = new LightLocalizer(odometer,leftMotor, rightMotor,navigation);
			//lightLocalizer.goToPoint();
			lightLocalizer.localise();
		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}
