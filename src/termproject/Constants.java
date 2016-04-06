package termproject;

/**
 * This class is a container class for all the constant values for this project
 * @author Stuart Mashaal
 *
 */
public class Constants {
	//values given to robot through wifi at beginning of round
	public static int starting_corner;
	public static int role;
	public static int goal_width;
	public static int defender_line;
	public static int forward_line;
	public static int ll_x;
	public static int ll_y;
	public static int ur_x;
	public static int ur_y;
	
	
/////////////////			ROBOT PHYSICAL DIMENSIONS AND PORTS             /////////////////////
	/**
	 * The distance between the two wheels of the robot
	 */
	public static final double WB = 16.0;	
	
	/**
	 * The radius of the robot's wheels
	 */
	public static final double WR = 1.9980 ;
	
	/**
	 * The distance, in cm, from the back-most point of the robot to the wheels
	 */
	public static final double WHEELS_TO_BACK = 17.8;
	
	/**
	 * The distance, in cm, from the sensor to the wheels
	 */
	public static final double WHEELS_TO_USSENSOR = 8.0;
	
	/**
	 * The distance, in cm, from the point between the robots wheels to the point that the light sensor is at on the ground
	 */
	public static final double WHEELCENTER_TO_LIGHTSENSOR = 7.5; //TODO: find actual value
	
	/**
	 * The angle, in degrees, between the angle of the direction perfectly to the right (90) and the angle from wheelcenter to lightsensor
	 */
	public static final double LIGHTSENSOR_ANGLE_OFFSET = 10; //TODO: find actual value
	
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
	
	
/////////////////			MOTOR SPEEDS AND ACCELERATIONS			////////////////////
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

///////////////							CODE TIMING			////////////////////////////
	/**
	 * The time in between every recalculation of position and heading by the Odometer, in ms
	 */
	public static final int ODOMETER_PERIOD = 40;
	
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
	public static final int LCD_PERIOD = 100;
	
	/**
	 * The time that the run-loop sleeps between every iteration
	 */
	public static final int RUNLOOP_PERIOD = 30;
	
/////////////////			FIELD DIMENSIONS			/////////////////////
	/**
	 * The side length of one square tile on the "playing field"
	 */
	public static final double TILE_LENGTH = 30.485;
	
	/**
	 * The length of one side of the playing field, measured in number of tiles.
	 * We assume that the playing-field is a square.
	 */
	public static final int PF_SIDE_LENGTH = 8; //for "training map" in dpm lab, 8x8 but for actual competition its 12x12
	
	
////////////////			SENSOR-RELATED CONSTANTS			///////////////////
	/**
	 * The max value to accept from the US poller
	 * All values greater than this will be set to this
	 */
	public static final double MAX_US_FILTER = 80.0;
	
	/**
	 * The color sensor value below which we consider a black line to have been detected
	 */
	public static final double COLOR_THRESHOLD =  45; //TODO find actual value
	
	/**
	 * The US sensor value below which we consider having to use our obstacle avoidance routine
	 */
	public static final double EMERGENCY_DISTANCE = 20.0;
	
	/**
	 * how reliable, (0 < reiablity < 11.0)/1.0, we expect sensor and other data to be
	 */
	public static final double RELIABILITY_FACTOR = 0.97;
}
