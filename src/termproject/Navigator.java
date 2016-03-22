package termproject;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class contains all the high-level methods for controlling the motion of
 * the robot on the playing field. Positive angle is counter-clockwise, positive distance is forward.
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
		rotateByDeg(Math.toDegrees(theta));
	}
	/**
	 * rotate the robot by a certain angle, in degrees
	 * @param theta the angle, in degrees, to rotate the robot by
	 */
	public void rotateByDeg(double theta) {
		setMotorSpeeds(Constants.ROTATESPEED);
		int wheelAngle = (int)(theta * Constants.WB * 0.5 / Constants.WR);
		leftMotor.rotate(wheelAngle, true);
		rightMotor.rotate(wheelAngle, false);
	}
	/**
	 * rotate the robot to a certain heading specified in radians
	 * @param theta the heading, in radians, to rotate to
	 */
	public void rotateToRad(double theta) {
		rotateToDeg(Math.toDegrees(theta));
	}
	/**
	 * rotate the robot to a certain heading specified in degrees
	 * @param theta the heading, in degrees, to rotate to
	 */
	public void rotateToDeg(double theta) {
		//TODO: make sure this works, doing the subtraction and then taking mod 360
		theta -= odo.getThetaDeg();
		theta %= 360; 
		//theta is now the angle to rotate by, in range [-360,360]
		
		if (theta < -180) {
			theta += 360;
		} else if (theta > 180) {
			theta -= 360;
		}
		//theta is now the angle to rotate by, in range [-180,180]
		
		rotateByDeg(theta);
	}
	
	/**
	 * move the robot forward the desired number of cm
	 * @param dist the distance, in cm, to move the robot forward by
	 */
	public void forwardBy(double dist) {
		int wheelAngle = (int)(Math.toDegrees(dist / Constants.WR));
		leftMotor.rotate(wheelAngle);
		rightMotor.rotate(wheelAngle);
	}
	/**
	 * move the robot backward the desired number of cm
	 * @param dist the distance, in cm, to move the robot backward by
	 */
	public void backwardBy(double dist) {
		forwardBy(-dist);
	}
	
	/**
	 * Sets the speed of the motors
	 * @param speed The speed, in degrees per second, to set the motor speed to
	 */
	public void setMotorSpeeds(int speed) {
		leftMotor.setSpeed(speed);
		rightMotor.setSpeed(speed);
	}
	
	
}
