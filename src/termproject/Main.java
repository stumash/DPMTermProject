package termproject;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.utility.Timer;

/**
 * This class contains the main method and all the high-level gameplay logic
 * @author Stuart Mashaal and Mathieu Savoie
 *
 */
public class Main {
	//member variables
	private enum State {INIT, LOCALIZE, GOTOSTART,
		GETBALL, GOTONET, PREPARESHOT, FIRE, 
		DEFEND, TRAVELLING, OBSTACLEAVOIDANCE};
	private static State state = State.LOCALIZE; //the current behavior state
	private static State prevState; //the previous behavior state
	boolean moving;
	
	//resources
	private static EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort(Constants.LEFT_MOTOR_PORT));
	private static EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort(Constants.RIGHT_MOTOR_PORT));
	private static EV3LargeRegulatedMotor sensorMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort(Constants.US_MOTOR_PORT));
	
	private static Odometer odo = new Odometer(leftMotor, rightMotor);
	private static Timer odoTimer = new Timer(Constants.ODOMETER_PERIOD, odo);
	
	private static USPoller usp = new USPoller(LocalEV3.get().getPort(Constants.US_PORT), sensorMotor);
	private static Timer usTimer = new Timer(Constants.US_PERIOD, usp);
	
	private static ColorPoller cp = new ColorPoller(LocalEV3.get().getPort(Constants.COLOR_PORT));
	private static Timer colorTimer = new Timer(Constants.COLOR_PERIOD, cp);
	
	private static LCDinfo lcd = new LCDinfo(odo);
	private static Timer lcdTimer = new Timer(Constants.LCD_PERIOD, lcd);
	
	private static Navigator nav = new Navigator(leftMotor, rightMotor, odo);
	
	private static Localizer loc = new Localizer(nav, usp, odo);
	
	public static void main(String[] args) {
		//start all timers
		odoTimer.start();
		usTimer.start();
		colorTimer.start();
		lcdTimer.start();
		try{ Thread.sleep(1500); }catch(InterruptedException e){e.printStackTrace();} //take a moment for all the timers to start up
		
		//the main run-loop
		runloop:
		while (true) {
			switch (state) {
			case INIT:
				loc.setMyCornerCoords(new double[] {0,0,0}); //TODO create the value-setting logic for this
				updateState(State.LOCALIZE);
				break;
			case LOCALIZE:
				loc.localize();
				updateState(State.GOTOSTART);
				break;
			case GOTOSTART:
				nav.goTo(Constants.TILE_LENGTH * 2, Constants.TILE_LENGTH * 2);
				break runloop;
			case TRAVELLING:
				break;
			case DEFEND:
				break;
			case GETBALL:
				break;
			case GOTONET:
				break;
			case PREPARESHOT:
				break;	
			case FIRE:
				break;
			case OBSTACLEAVOIDANCE:
				break;		
			}
			
			try { //time in between every execution of the run-loop
				Thread.sleep(Constants.RUNLOOP_PERIOD);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
	
	//sets prevState to state and then sets state to s
	private static void updateState(State s) {
		prevState = state;
		state = s;
	}
}
