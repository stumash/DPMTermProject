package termproject;

/**
 * This class is a container class for all the constant values for this project
 * @author Stuart Mashaal and Mathieu Savoie
 *
 */
public class Constants {
//ROBOT PHYSICAL DIMENSIONS
	/**
	 * The distance between the two wheels of the robot
	 */
	public static final double WB=16.0;	
	
	/**
	 * The radius of the robot's wheels
	 */
	public static final double WR=2.7;
	
	/**
	 * The distance, in the x direction, from the center of the wheel base to the line sensor
	 */
	public static final double LINESENSOR_XOFFEST = 4.5;
	
	/**
	 * The distance,  in the y direction, from the center of the wheel base to the line sensor
	 */
	public static final double LINESENSOR_YOFFSET = 4.5;
	
//MOTOR SPEEDS AND ACCELERATIONS
	/**
	 * The speed of the wheel, in degrees per second, when the robot rotates to change heading
	 */
	public static final int ROTATESPEED = 150;
	
	/**
	 * The speed of the wheels, in degrees per second, when the robot moves along the axis of its current heading
	 * (forward or backwards)
	 */
	public static final int GOSPEED = 250;
	
	/**
	 * The acceleration of the arm motor when acquiring a ball
	 */
	public static final int GETBALLARMACCELERATION = 3000;
	
	/**
	 * The acceleration of the arm motor when firing a ball
	 */
	public static final int FIREARMACCELERATION = 9000;
	
	/**
	 * The speed of the arm motor when acquiring a ball
	 */

//CODE TIMING
	/**
	 * The time in between every recalculation of position and heading by the Odometer
	 */
	public static final int ODOMETERPERIOD = 25;
	
	/**
	 * The time in between every update in the distance measurement recorded by the US sensor
	 */
	public static final int USPOLLERPERIOD = 50;
	
	/**
	 * The time i between every update in the light measurement recorded by the Color sensor
	 */
	public static final int COLORPOLLERPERIOD = 50;
}
