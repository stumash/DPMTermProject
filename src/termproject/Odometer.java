/*
 * Odometer.java
 */

package termproject;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.utility.TimerListener;

/**
 * 
 * @author Stuart Mashaal
 *
 */
public class Odometer implements TimerListener {

	//Class Variables
	private int lastTachoL;			// Tacho L at last sample
	private int lastTachoR;			// Tacho R at last sample 
	private int nowTachoL;			// Current tacho L
	private int nowTachoR;			// Current tacho R
	private double distL, distR, deltaD, deltaT, dX, dY;
	
	//robot position
	private double X;					/** Current X position, in cm */
	private double Y;					/** Current Y position, in cm */
	private double Theta;				/** Current orientation, in radians */
	
	//resources
	private EV3LargeRegulatedMotor rightMotor;
	private EV3LargeRegulatedMotor leftMotor;

	// lock object for mutual exclusion
	private Object lock;

	/**
	 * Constructs an Odometer instance with a reference to a leftMotor and a rightMotor
	 * @param leftMotor The motor controlling the robot's left wheel
	 * @param rightMotor The motor controlling the robot's right wheel
	 */
	public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		X = 0.0;
		Y = 0.0;
		Theta = 0.0;
		lock = new Object();
		
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		
		leftMotor.resetTachoCount();
	    rightMotor.resetTachoCount();
	    lastTachoL=leftMotor.getTachoCount();
	    lastTachoR=rightMotor.getTachoCount();
	}

	public void timedOut() {
		nowTachoL = leftMotor.getTachoCount();      		// get tacho counts
		nowTachoR = rightMotor.getTachoCount();
		distL = Math.PI*Constants.WR*(nowTachoL-lastTachoL)/180;		// compute L and R wheel displacements
		distR = Math.PI*Constants.WR*(nowTachoR-lastTachoR)/180;
		lastTachoL=nowTachoL;								// save tacho counts for next iteration
		lastTachoR=nowTachoR;
		deltaD = 0.5*(distL+distR);							// compute vehicle displacement
		deltaT = (distL-distR)/Constants.WB;							// compute change in heading

		synchronized (lock) {
			// don't use the variables x, y, or theta anywhere but here!
			Theta += deltaT;									// update heading
		    dX = deltaD * Math.sin(Theta);						// compute X component of displacement
			dY = deltaD * Math.cos(Theta);						// compute Y component of displacement
			X = X + dX;											// update estimates of X and Y position
			Y = Y + dY;	
		}
	}

	//Getters
	/**
	 * Changes 0th,1st,2nd values of position[] to X,Y,Theta, respectively
	 * @param position the array to copy X,Y,Theta into. Theta in radians.
	 */
	public void getPosition(double[] position) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			position[0] = X;
			position[1] = Y;
			position[2] = Theta;
		}
	}
	
	/**
	 * getter for X coordinate of robot
	 * @return the most recent X coordinate of robot's position calculated by Odometer, im cm
	 */
	public double getX() {
		double result;

		synchronized (lock) {
			result = X;
		}

		return result;
	}

	/**
	 * getter for Y coordinate of robot
	 * @return the most recent Y coordinate of robot's position calculated by Odometer, in cm
	 */
	public double getY() {
		double result;

		synchronized (lock) {
			result = Y;
		}

		return result;
	}

	/**
	 * getter for Theta coordinate of robot
	 * @return the most recent Theta coordinate of robot's position calculated by Odometer, in radians
	 */
	public double getThetaRad() {
		double result;

		synchronized (lock) {
			result = Theta;
		}

		return result;
	}
	/**
	 * getter for Theta coordinate of robot
	 * @return the most recent Theta coordinate of robot's position calculated by Odometer, in degrees
	 */
	public double getThetaDeg() {
		double result;

		synchronized (lock) {
			result = Math.toDegrees(Theta);
		}

		return result;
	}
	/**
	 * getter for Theta coordinate of robot, but always given in range [0,360]
	 * @returns the most recent Theta coordinate calculated by the Odometer, in degrees and in range [0,360]
	 */
	public double getThetaDeg_positive() {
		double result = getThetaDeg() % 360;
		if (result < 0) {
			result += 360;
		}
		return result;
	}

	//Setters
	
	/**
	 * Sets X,Y,Theta to 0th,1st,2nd values of position[], respectively
	 * @param position the array to copy to X,Y,Theta. Values for Theta should be in radians.
	 */
	public void setPosition(double[] position) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			this.X = position[0];
			this.Y = position[1];
			this.Theta = position[2];
		}
	}
	/**
	 * Sets X,Y,Theta to 0th,1st,2nd values of position[], respectively
	 * @param position the array to copy to X,Y,Theta. Values for Theta should be in deg.
	 */
	public void setPositionDeg(double[] position) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			this.X = position[0];
			this.Y = position[1];
			this.Theta = Math.toRadians(position[2]);
		}
	}

	/**
	 * Sets the X value of the Odometer to x
	 * @param x the value to set Odometer.X to, in cm
	 */
	public void setX(double x) {
		synchronized (lock) {
			this.X = x;
		}
	}

	/**
	 * Sets the Y value of the Odometer to y
	 * @param Y the value to set Odometer.Y to, in cm
	 */
	public void setY(double y) {
		synchronized (lock) {
			this.Y = y;
		}
	}

	/**
	 * Sets the Theta value of the Odometer to theta
	 * @param theta the value to set Odometer.Theta to, in radians
	 */
	public void setThetaRad(double theta) {
		synchronized (lock) {
			this.Theta = theta;
		}
	}
	/**
	 * Sets the Theta value of the Odometer to theta
	 * @param theta the value to set the Odometer.Theta to, in degrees
	 */
	public void setThetaDeg(double theta) {
		setThetaRad(Math.toRadians(theta));
	}
}