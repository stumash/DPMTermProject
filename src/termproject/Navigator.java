package termproject;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * this class contains all the high-level methods for controlling the motion of
 * the robot on the playing field
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
	 * @param odo the odometer to give the navigator the robot's current position and heading so
	 * it can move and change the robot's heading relative to current values and not just by absolute amounts
	 */
	public Navigator(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odo) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odo = odo;
	}
	
	/**
	 * 
	 * @param x
	 * @param y
	 */
	public void goTo(double x, double y) {
		//TODO
	}
	
	/**
	 * rotate the robot by a certain angle, in radians
	 * @param theta the angle, in radians, to rotate the robot by
	 */
	public void rotateByRad(double theta) {
		//TODO
	}
	/**
	 * rotate the robot by a certain angle, in degrees
	 * @param theta the angle, in degrees, to rotate the robot by
	 */
	public void rotateByDeg(double theta) {
		rotateByRad(Math.toRadians(theta));
	}
	/**
	 * rotate the robot to a certain heading specified in radians
	 * @param theta the heading, in radians, to rotate to
	 */
	public void rotateToRad(double theta) {
		rotateByRad(theta - odo.getThetaRad());
	}
	/**
	 * rotate the robot to a certain heading specified in degrees
	 * @param theta the heading, in degrees, to rotate to
	 */
	public void rotateToDeg(double theta) {
		rotateToRad(Math.toRadians(theta));
	}
	
	
}
