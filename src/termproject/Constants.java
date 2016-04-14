package termproject;

/**
 * This class is a container class for all the constant values for this project
 * @author Stuart Mashaal
 *
 */
public class Constants {
	//values given to robot through wifi at beginning of round
	public static int goal_width;
	public static double defender_line; //distance from back wall of playing field to defense-zone front line
	public static double forward_line; //distance from front wall of playing field to offense-zone front line
	public static double ll_x; //lower-left and upper-right xy coordinate values for ball holder
	public static double ll_y;
	public static double ur_x;
	public static double ur_y;
	public static int ballQty; //number of balls on playing field
	public static double balldest_x; //x-coord for where to light localize before getting balls
	public static double balldest_y; //y-coord for where to light localize before getting balls
	public static double startx, starty;
	public static double firedest_x, firedest_y, firedest_th;
	
	
/////////////////			ROBOT PHYSICAL DIMENSIONS         /////////////////////
	/**
	 * The distance between the two wheels of the robot
	 */
	public static final double WB = 16.7;	
	
	/**
	 * The radius of the robot's wheels
	 */
	public static final double WR = 2.028;
	
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
	public static final double WHEELCENTER_TO_LIGHTSENSOR = 12.5;
	
	/**
	 * The angle, in degrees, between the angle of the direction perfectly to the right (90) and the angle from wheelcenter to lightsensor
	 */
	public static final double LIGHTSENSOR_ANGLE_OFFSET = 31.0; 
	
	/**
	 * The angle the arm must rotate from its reset position in order to tap the ball towards the ball holder
	 */
	public static final int GETBALL_TAPANGLE = 160;
	
	/**
	 * The angle the arm must rotate (after tapping the ball towards the holder) to push the ball
	 * all the way into the ball holder
	 */
	public static final int GETBALL_HOLDERPUSHANGLE = 50;
	
	/**
	 * The angle the arm must rotate to wind up backwards to be in position to suddenly rotate very fast forward
	 * to slap the ball into the goal
	 */
	public static final int FIRE_WINDUPANLGE = -150;
	
	/**
	 * The angle the arm must rotate forward at high speed to fire the ball at the net
	 */
	public static final int FIRE_SLAPSHOTANGLE = 175;
	
	/**
	 * The angle, clockwise, that the ball firing procedure aims off the expected perfect 90 degrees
	 */
	public static final double FIRE_ERROR_OFFSET_ANGLE = 10;
	
	
////////////////////////////// 			ROBOT PORTS				///////////////////////////
	/**
	 * The port identification string for the left wheel motor
	 */
	public static final String LEFT_MOTOR_PORT = "A";
	
	/**
	 * The port identification string for the right wheel motor
	 */
	public static final String RIGHT_MOTOR_PORT = "D";
	
	/**
	 * The port identification string for the US sensor's motor
	 */
	public static final String US_MOTOR_PORT = "C";
	
	/**
	 * The port identification string for the arm's motor
	 */
	public static final String ARM_MOTOR_PORT = "B";
	
	/**
	 * The port identification string for the US sensor
	 */
	public static final String US_PORT = "S3";
	
	/**
	 * The port identification string for the light sensor
	 */
	public static final String LIGHT_PORT = "S2";
	
	/**
	 * The port identification string for the color sensor
	 */
	public static final String COLOR_PORT = "S4"; //TODO: make sure this is correct
	
	
/////////////////			MOTOR SPEEDS AND ACCELERATIONS			////////////////////
	/**
	 * The speed of the wheel, in degrees per second, when the robot rotates to change heading
	 */
	public static final int ROTATE_SPEED = 170;
	
	/**
	 * The speed of the wheels, in degrees per second, when the robot moves along the axis of its current heading
	 * (forward or backwards)
	 */
	public static final int GO_SPEED = 250;
	
	/**
	 * The speed of the wheel, in degrees per second, when the robot backs into the wall during localization
	 */
	public static final int LOCALIZE_BACKUP_SPEED = 275;
	
	/**
	 * The speed of the arm motor, in deg/sec, when acquiring a ball 
	 */
	public static final int GETBALL_ARMSPEED = (int)(160/2.35);
	
	/**
	 * The speed at which the ball-arm nudges the ball into the holder after having knocked it out
	 */
	public static final int GETBALL_PUSHSPEED = 250;
	
	/**
	 * The acceleration of the arm motor when acquiring a ball
	 */
	public static final int GETBALL_ARMACCELERATION = 3000;
	
	/**
	 * The acceleration of the arm motor when firing a ball
	 */
	public static final int FIRE_ARMACCELERATION = 11000;
	
	/**
	 * The speed of the arm motor when acquiring a ball
	 */
	public static final int FIRE_ARMSPEED = 1100;
	
	/**
	 * The speed of the arm motor when resetting to its start position after firing a shot
	 */
	public static final int ARMRESET_SPEED = 600;
	

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
	public static final int LIGHT_PERIOD = 40;
	
	/**
	 * The time in between every update of the info displayed by the LCD screen
	 */
	public static final int LCD_PERIOD = 200;
	
	/**
	 * The time that the run-loop sleeps between every iteration, in ms
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
	public static final double LIGHT_THRESHOLD =  45;
	
	/**
	 * The US sensor value below which we consider having to use our obstacle avoidance routine
	 */
	public static final double EMERGENCY_DISTANCE = 20.0;
	
	/**
	 * how reliable, (0 < reiablity < 11.0)/1.0, we expect sensor and other data to be
	 */
	public static final double RELIABILITY_FACTOR = 0.95;
	
	/**
	 * our team's team number for DPM Winter 2016
	 */
	public static final int TEAM_NUMBER = 10;
	
	/**
	 * The ip adress of the server that the robot connects to get setup values
	 */
	public static final String SERVER_IP = "192.168.10.200";
	
	/**
	 * The number of centimeters in an inch
	 */
	public static final double INCH = 2.54;
}
