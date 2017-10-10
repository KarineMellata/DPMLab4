import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * 
 * This class contains the main method. It gives the user the choice between
 * a falling-edge method of ultrasonic localization or a rising-edge. The robot will then
 * first roughly localize itself (find out its orientation) using the US method. It will wait
 * for a button press and then the light localizer will start and localize the robot on the map (X and Y).
 * @author Karine Mellata and Mustafa Khawaja
 * @param  Odometer class
 * @param  Navigation class
 * @param  OdemetryDisplay class
 * @param  UltrasonicLocalizer class
 * @param  LightLocalizer class
 * @param  UltrasonicPoller class
 *
 */

public class LocalisationLab {
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3UltrasonicSensor uSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S4"));
	public static final double WHEEL_RADIUS = 2.1;
	public static final double TRACK = 12.5;
	public static final double TM = 30.48;
	public enum LocalizationType {F_E, R_E};


	public static void main(String[] args) {
		//Setting up objects
		int buttonChoice;
		final TextLCD t = LocalEV3.get().getTextLCD();
		Odometer odometer = new Odometer(leftMotor, rightMotor);
		Navigation navigation = new Navigation(odometer, leftMotor, rightMotor, WHEEL_RADIUS, TRACK);
		@SuppressWarnings("resource")
		SensorModes us = uSensor;
		SampleProvider usDistance = us.getMode("Distance");	// usDistance provides samples from this instance of the sensor
		float[] usData = new float[usDistance.sampleSize()];		// usData is the buffer in which data are returned
		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, t);
		do {
			// clear the display
			t.clear();

			// ask the user whether the motors should localise in rising-edge or falling-edge
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
			lightLocalizer.localise();
		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}
