package termproject;

/**
 * This class is a container class for all the constant values for this project
 * @author Stuart Mashaal and Mathieu Savoie
 *
 */
public class Constants {
//ROBOT PHYSICAL DIMENSIONS AND PORTS
	/**
	 * The distance between the two wheels of the robot
	 */
	public static final double WB = 16.0;	
	
	/**
	 * The radius of the robot's wheels
	 */
	public static final double WR = 1.9975;
	
	/**
	 * The distance, in cm, from the back-most point of the robot to the wheels
	 */
	public static final double WHEELS_TO_BACK = 17.8;
	
	/**
	 * The distance, in the x direction, from the center of the wheel base to the line sensor
	 */
	public static final double LINESENSOR_XOFFEST = 4.5;
	
	/**
	 * The distance,  in the y direction, from the center of the wheel base to the line sensor
	 */
	public static final double LINESENSOR_YOFFSET = 4.5;
	
	/**
	 * The port identification string for the left wheel motor
	 */
	public static final String LEFT_MOTOR_PORT = "A";
	
	/**
	 * The port identification string for the right wheel motor
	 */
	public static final String RIGHT_MOTOR_PORT = "D";
	
	/**
	 * The port identification string for the US sensor
	 */
	public static final String US_PORT = "S3";
	
	/**
	 * The port identification string for the Color sensor
	 */
	public static final String COLOR_PORT = "S1";
	
	/**
	 * The port identification string for the US sensor's motor
	 */
	public static final String US_MOTOR_PORT = "C";
	
//MOTOR SPEEDS AND ACCELERATIONS
	/**
	 * The speed of the wheel, in degrees per second, when the robot rotates to change heading
	 */
	public static final int ROTATE_SPEED = 150;
	
	/**
	 * The speed of the wheels, in degrees per second, when the robot moves along the axis of its current heading
	 * (forward or backwards)
	 */
	public static final int GO_SPEED = 150;
	
	/**
	 * The speed of the wheel, in degrees per second, when the robot backs into the wall during localization
	 */
	public static final int LOCALIZE_BACKUP_SPEED = 275;
	
	/**
	 * The acceleration of the arm motor when acquiring a ball
	 */
	public static final int GETBALL_ARMACCELERATION = 3000;
	
	/**
	 * The acceleration of the arm motor when firing a ball
	 */
	public static final int FIRE_ARMACCELERATION = 9000;
	
	/**
	 * The speed of the arm motor when acquiring a ball
	 */

//CODE TIMING
	/**
	 * The time in between every recalculation of position and heading by the Odometer, in ms
	 */
	public static final int ODOMETER_PERIOD = 25;
	
	/**
	 * The time in between every update in the distance measurement recorded by the US sensor, in ms
	 */
	public static final int US_PERIOD = 40;
	
	/**
	 * The time in between every update in the color measurement of the color sensor, in ms
	 */
	public static final int COLOR_PERIOD = 40;
	
	/**
	 * The time in between every update of the info displayed by the LCD screen
	 */
	public static final int LCD_PERIOD = 60;
	
	/**
	 * The time that the run-loop sleeps between every iteration
	 */
	public static final int RUNLOOP_PERIOD = 30;
	
//OTHER
	/**
	 * The side length of one square tile on the "playing field"
	 */
	public static final double TILE_LENGTH = 30.485;
	
	/**
	 * The max value to accept from the US poller
	 * All values greater than this will be set to this
	 */
	public static final double MAX_US_FILTER = 60.0;
	
}
