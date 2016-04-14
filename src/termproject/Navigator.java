package termproject;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class contains all the high-level methods for controlling the motion of
 * the robot on the playing field. Positive angle is counter-clockwise, positive distance is forward.
 * @author Stuart Mashaal
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
	 * rotate to right or left if xdest > odo.getX() or xdest < odo.getTheta(), respectively
	 * @return true if xdest > odo.get() else false
	 */
	public boolean rotateToXdest(double xdest) {
		if (odo.getX() >= xdest) { //if you're to the right of xdest
			rotateToDeg(-90); //rotate to left
			return true;
		} else {
			rotateToDeg(90); //rotate to right
			return false;
		}
	}
	/**
	 * rotate to 0 or 180 if ydest > odo.getY() or ydest < odo.getY(), respectively
	 * @return true if ydest > odo.get() else false
	 */
	public boolean rotateToYdest(double ydest) {
		if (ydest >= odo.getY()) {
			rotateToDeg(0);
			return true;
		} else {
			rotateToDeg(180);
			return false;
		}
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
	/**
	 * go forward indefinitely. method call returns immediately, allowing
	 * any code that follows to be executed while robot is going forward
	 */
	public void goForward_imret() {
		//go forward by arbitrarily large distance
		forwardBy_imret(Constants.TILE_LENGTH * 30);
	}
	/**
	 * go backward indefinitely. method call returns immediately, allowing
	 * any code that follows to be executed while robot is going backward
	 */
	public void goBackward_imret() {
		backwardBy_imret(Constants.TILE_LENGTH * 30);
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
		leftMotor.setSpeed(0);
		rightMotor.setSpeed(0);
		leftMotor.forward();
		rightMotor.forward();
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
	
	/**
	 * determines the distance to the wall depending if robot is facing up, down, left or right
	 * @return the distance in cm to the wall that the robot is facing
	 */
	public double distToWall() {
		double retval;
		double th = ((odo.getThetaDeg() % 360) + 360) % 360; //th = current heading in range [0, 360]
		if (th > 315 || th <= 45 ) { //if facing upwards
			retval = Constants.TILE_LENGTH * (Constants.PF_SIDE_LENGTH - 1) - odo.getY(); //return distance to top wall
		} else if (th <= 135) {//if facing to the right
			retval = Constants.TILE_LENGTH * (Constants.PF_SIDE_LENGTH - 1) - odo.getX(); //return distance to right wall
		} else if (th <= 225) {//if facing downward
			retval = Constants.TILE_LENGTH + odo.getY(); //return distance to bottom wall
		} else {//else facing left
			retval =  Constants.TILE_LENGTH + odo.getX(); //return distance to left wall
		}
		return retval;
	}
	
	/**
	 * determines the coordinates of the nearest groundline intersection usable for light localization
	 * @return an array of two values containing the x and y coordinate of the groundline intersection
	 * nearest the x,y coordinate provided as argument
	 */
	public double[] nearestIntersectionCoords() {
		double x = odo.getX();
		double y = odo.getY();
		double[] result = new double[2];
		double rem = 0;
		if ((rem = x % Constants.TILE_LENGTH) < Constants.TILE_LENGTH / 2) { 
			//the nearest intersection is at lower x coord
			result[0] = odo.getX() - rem;
		} else {
			result[0] = odo.getX() + (Constants.TILE_LENGTH - rem);
		}
		if((rem = y % Constants.TILE_LENGTH) < Constants.TILE_LENGTH / 2) {
			//the nearest intersection is at lower y coord
			result[1] = odo.getY() - rem;
		} else {
			result[1] = odo.getY() + (Constants.TILE_LENGTH - rem);
		}
		return result;
	}
	
	/**
	 * determines if the robot is within the specified distance to the nearest groundline intersection
	 * @param dist the distance, in cm
	 * @return true if the robot is within the specified distance the nearest groundline intersection, else false
	 */
	public boolean isWithinDistOfNearestIntersection(double dist) {
		double[] nearest = nearestIntersectionCoords();
		return Math.sqrt(Math.pow(nearest[0] - odo.getX(), 2) + Math.pow(nearest[1] - odo.getY(), 2)) < dist;
	}
}
