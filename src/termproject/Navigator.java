package termproject;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * this class contains all the methods for the controlling the robot's heels and thuus motios
 * @author Stuart Mashaal and Mathieu Savoie
 *
 */
public class Navigator {
	EV3LargeRegulatedMotor leftMotor;
	EV3LargeRegulatedMotor rightMotor;
	Odometer odo;
	

	/**
	 * constructs a Navigator instance
	 * @param leftMotor the motor of the left wheel
	 * @param rightMotor the motor of the right wheel
	 * @param odo the odometer so that the navigator can receive command for relative motion
	 */
	public Navigator(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odo) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odo = odo;
	}
}
