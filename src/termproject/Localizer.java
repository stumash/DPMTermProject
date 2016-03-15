package termproject;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class is for getting the robot to figure out where it is.  It
 * has only one method, 'localize'.
 * @author Stuart Mashaal and Mathieu Savoie
 *
 */
public class Localizer {
	EV3LargeRegulatedMotor leftMotor;
	EV3LargeRegulatedMotor rightMotor;
	USPoller usp;
	ColorPoller cp;

	/**
	 * constructs a Localizer instance with referenced to to wheel motors (left and right),
	 * an UltraSonic Poller and a Color Poller.
	 * @param leftMotor the robot's left motor
	 * @param rightMotor the robot's right motor
	 * @param usp the ultrasonic poller for distance-to-wall detection
	 * @param cp the color poller for line detection
	 */
	public Localizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, USPoller usp, ColorPoller cp) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.usp = usp;
		this.cp = cp;
	}
	
	/**
	 * executes localization routine
	 */
	public void localize() {
		
	}
}
