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
	 * the robot will travel to the point (x,y) regardless of current location
	 * @param x the x coordinate to travel to
	 * @param y the y coordinate to travel to
	 */
	public void goTo(double x, double y) {
		//travel horizontal distance, then vertical distance
		double xdist = x - odo.getX();
		double ydist = y - odo.getY();
		if (xdist < 0) {
			rotateToDeg(-90);
			forwardBy(Math.abs(xdist));
		} else if (xdist > 0) {
			rotateToDeg(90);
			forwardBy(xdist);
		}
		if (ydist < 0) {
			rotateToDeg(180);
			forwardBy(Math.abs(ydist));
		} else if (ydist > 0) {
			rotateToDeg(0);
			forwardBy(ydist);
		}
		
	}

//rotation methods
	/**
	 * rotate the robot by a certain angle, in radians
	 * @param theta the angle, in radians, to rotate the robot by
	 */
	public void rotateByRad(double theta) {
		rotateByDeg(Math.toDegrees(theta));
	}
	/**
	 * rotate the robot by a certain angle, in radians. Method call returns immediately, regardless
	 * of whether or not the motion is completed yet
	 * @param theta the angle, in radians, to rotate the robot by
	 */
	public void rotateByRad_imret(double theta) {
		rotateByDeg_imret(Math.toDegrees(theta));
	}
	/**
	 * rotate the robot by a certain angle, in degrees
	 * @param theta the angle, in degrees, to rotate the robot by
	 */
	public void rotateByDeg(double theta) {
		setMotorSpeeds(Constants.ROTATE_SPEED);
		int wheelAngle = (int)(theta * Constants.WB * 0.5 / Constants.WR);
		leftMotor.rotate(wheelAngle, true);
		rightMotor.rotate(-wheelAngle, false);
	}
	/**
	 * rotate the robot by s certain angle, in degrees. Method call returns immediately, regardless
	 * of whether or not the motion is completed yet
	 * @param theta the angle, in degrees, to rotate the robot by
	 */
	public void rotateByDeg_imret(double theta) {
		setMotorSpeeds(Constants.ROTATE_SPEED);
		int wheelAngle = (int)(theta * Constants.WB * 0.5 / Constants.WR);
		leftMotor.rotate(wheelAngle, true);
		rightMotor.rotate(-wheelAngle, true);
	}
	/**
	 * rotate the robot to a certain heading specified in radians
	 * @param theta the heading, in radians, to rotate to
	 */
	public void rotateToRad(double theta) {
		rotateToDeg(Math.toDegrees(theta));
	}
	/**
	 * rotate the robot to a certain heading specified in radians. Method call returns immediately, regardless
	 * of whether or not the motion is completed yet
	 * @param theta the heading, in radians, to rotate to
	 */
	public void rotateToRad_imret(double theta) {
		rotateToDeg_imret(Math.toDegrees(theta));
	}
	/**
	 * rotate the robot to a certain heading specified in degrees
	 * @param theta the heading, in degrees, to rotate to
	 */
	public void rotateToDeg(double theta) {
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
	 * rotate the robot to a certain heading specified in degrees. Method call returns immediately, regardless
	 * of whether or not the motion is completed yet
	 * @param theta the heading, in degrees, to rotate to
	 */
	public void rotateToDeg_imret(double theta) {
		theta -= odo.getThetaDeg();
		theta %= 360; 
		//theta is now the angle to rotate by, in range [-360,360]
		
		if (theta < -180) {
			theta += 360;
		} else if (theta > 180) {
			theta -= 360;
		}
		//theta is now the angle to rotate by, in range [-180,180]
		
		rotateByDeg_imret(theta);
	}
	
	/**
	 * rotate the robot in the clockwise direction until stop() is called
	 */
	public void rotateClockwiseContinuous() {
		setMotorSpeeds(Constants.ROTATE_SPEED);
		leftMotor.forward();
		rightMotor.backward();
	}
	/**
	 * rotate the robot in the counter-clockwise direction until stop() is called
	 */
	public void rotateCounterclockiseContinuous() {
		setMotorSpeeds(Constants.ROTATE_SPEED);
		leftMotor.backward();
		rightMotor.forward();
	}
	
//forward and backward motion methods
	/**
	 * move the robot forward the desired number of cm
	 * @param dist the distance, in cm, to move the robot forward by
	 */
	public void forwardBy(double dist) {
		setMotorSpeeds(Constants.GO_SPEED);
		int wheelAngle = (int)(Math.toDegrees(dist / Constants.WR));
		leftMotor.rotate(wheelAngle, true);
		rightMotor.rotate(wheelAngle, false);
	}
	/**
	 * move the robot forward the desired number of cm. Method call returns immediately regardless
	 * of whether or not motion is completed yet
	 * @param dist
	 */
	public void forwardBy_imret(double dist) {
		setMotorSpeeds(Constants.GO_SPEED);
		int wheelAngle = (int)(Math.toDegrees(dist / Constants.WR));
		leftMotor.rotate(wheelAngle, true);
		rightMotor.rotate(wheelAngle, true);
	}
	/**
	 * move the robot backward the desired number of cm
	 * @param dist the distance, in cm, to move the robot backward by
	 */
	public void backwardBy(double dist) {
		forwardBy(-dist);
	}
	/**
	 * move the robot backward the desired number of cm. Method call returns immediately regardless 
	 * of whether or not motion is completed yet
	 * @param dist
	 */
	public void backwardBy_imret(double dist) {
		forwardBy_imret(-dist);
	}
	
//miscellaneous helper methods
	/**
	 * Sets the speed of the motors
	 * @param speed The speed, in degrees per second, to set the motor speed to
	 */
	public void setMotorSpeeds(int speed) {
		leftMotor.setSpeed(speed);
		rightMotor.setSpeed(speed);
	}
	
	/**
	 * Stops both motors
	 */
	public void stop() {
		leftMotor.stop();
		rightMotor.stop();
	}
	
	/**
	 * Backs the robot into a wall at an increased speed for localizatoin
	 */
	public void localizerBackup() {
		setMotorSpeeds(Constants.LOCALIZE_BACKUP_SPEED);
		backwardBy(20);
	}
	
	/**
	 * determines if the robot's wheels are currently attempting to rotate
	 * @return true if the wheels are attempting to rotate and false if not
	 */
	public boolean isMoving() {
		return leftMotor.isMoving() || rightMotor.isMoving();
	}
}
