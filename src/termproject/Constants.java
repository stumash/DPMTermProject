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
	 * The time between each odometer value update
	 */
	
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

//CODE TIMING
	/**
	 * The time in between every recalculation of position and heading by the Odometer
	 */
	public static final int ODOMETERPERIOD = 25;	
}
