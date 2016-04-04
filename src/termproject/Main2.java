package termproject;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.utility.Timer;

/**
 * This class contains the main method and all the high-level gameplay logic
 * @author Stuart Mashaal
 *
 */
public class Main2 {
	//member variables
	private enum State {INIT, LOCALIZE, GOTOSTART,
		GOTOBALLS, PICKUPBALL, GOTONET, PREPARESHOT, FIRE, 
		DEFEND, TRAVELLING, OBSTACLEAVOIDANCE};
	private static State currState = State.INIT; //the current behavior state
	private static State nextState; //the previous behavior state
	private static double xdest, ydest;
	private static double filteredusD;
	private static boolean homestretch;

	
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
		
		//take a moment for all the timers to start up
		try{ Thread.sleep(1500); }
		catch(InterruptedException e){e.printStackTrace();}

		//the main run-loop
		runloop:
		while (true) {
			
			//run-loop period implemented
			try{ Thread.sleep(Constants.RUNLOOP_PERIOD); }
			catch(InterruptedException e){e.printStackTrace();}
			
			stateswitch:
			switch (currState) {
			case INIT:
				nextState = State.LOCALIZE;
				
				//TODO create the value-setting logic for this using wifi classes
				
				loc.setCorner(1);
				
				currState = nextState;
				break;
			case LOCALIZE:
				nextState = State.GOTOSTART;
				
				//perform localization
				loc.localize();
				
				currState = nextState;
				break;
			case GOTOSTART:
				nextState = State.GOTOBALLS;
				
				
				
				xdest = Constants.TILE_LENGTH * 5; ydest = Constants.TILE_LENGTH * 5;
				
				
				
				currState = State.TRAVELLING;
				
				break;
			case DEFEND:
				break runloop;
			case GOTOBALLS:
				break runloop;
			case PICKUPBALL:
				break;
			case GOTONET:
				break;
			case PREPARESHOT:
				break;	
			case FIRE:
				break;
				
			/*
			 * the travelling state encapsulates all of the behaviour of the robot during traveling to 
			 * given point. the travelling state brings the robot to (xdest, ydest) while getting
			 * around all obstacles it encounters by using the intermediary state, obstacleavoidance.
			 */
			case TRAVELLING:
				//**take note that the travelling state does not assign a new value to nextState when entered
				
				//if you haven't yet completed the travelling in the x direction, 
				//you're not on the homestretch, which is y direction travelling since that's always after
				//x direction travelling finishes successfully.
				if (!homestretch) {
					
					//rotate towards xdest and then attempt to go to it
					nav.rotateToXdest(xdest);
					nav.forwardBy_imret(Math.abs(odo.getX() - xdest));
					
					//handle possibility of encountering obstacle
					while (nav.isMoving()) {
						//don't check too often 
						try{ Thread.sleep(Constants.RUNLOOP_PERIOD); }
						catch(InterruptedException e){e.printStackTrace();}
						
						//if you're approaching an obstacle
						if ((filteredusD = usp.getFilteredUSdistance()) < Constants.EMERGENCY_DISTANCE * Constants.RELIABILITY_FACTOR) { 
							if (filteredusD < nav.distToWall() * Constants.RELIABILITY_FACTOR) {
								currState = State.OBSTACLEAVOIDANCE; //go to obstacleavoidance state and stop
								nav.stop();							 //going towards (xdest,ydest)
								break stateswitch; 
							}
						}
					}
				
					//you travelled the xdistance in one piece. now do the y distance
					homestretch = true;
					
				//if you've finished the xdistance and are on the homestretch
				} else { 
				
					//rotate towards ydest and then attempt to go to it
					nav.rotateToYdest(ydest);
					nav.forwardBy_imret(Math.abs(ydest - odo.getY()));
				
					//handle the possibility of encountering obstacles
					while (nav.isMoving()) {
						//don't check too often 
						try{ Thread.sleep(Constants.RUNLOOP_PERIOD); }
						catch(InterruptedException e){e.printStackTrace();}
						
						//if you're approaching an obstacle
						if ((filteredusD = usp.getFilteredUSdistance()) < Constants.EMERGENCY_DISTANCE * Constants.RELIABILITY_FACTOR) {
							if (filteredusD < nav.distToWall() * Constants.RELIABILITY_FACTOR) {
								currState = State.OBSTACLEAVOIDANCE; //go to obstacleavoidance state and stop
								nav.stop();							 //going towards (xdest,ydest)
								break stateswitch; 
							}
						}
					}
					
					//you finished travelling. next time you start travelling again you won't be on the homestretch
					homestretch = false;
					
					//you've gotten to where you're going. now, do the next state whatever that is
					currState = nextState;					
				}
				
				break;

			/*
			 * the obstacleavoidance state encapsulates all robot's behaviour while it avoids/goes around
			 * an wooden block obstacle
			 */
			case OBSTACLEAVOIDANCE:
				/* take note that the obstacleavoidance state does not assign a new 
				 * value to nextState when entered
				 */				
				
				if (shouldGoAroundObstacleByTurningRight()) {
					//rotate to the right, aim sensor to the left and go until the obstacle is no longer to your left
					nav.rotateByDeg(90);
					usp.rotateByDeg(-90);
					nav.goForward_imret();
					
					//stop when the obstacle is no longer to your left
					while (usp.getFilteredUSdistance() < (Constants.EMERGENCY_DISTANCE) * (2 - Constants.RELIABILITY_FACTOR)) {
						//don't check too often
						try{ Thread.sleep(Constants.RUNLOOP_PERIOD); }
						catch(InterruptedException e){e.printStackTrace();}			
					}
					nav.stop();

					//then go some more so you can clear it when you turn back.  turn back
					nav.forwardBy(Constants.WHEELS_TO_BACK + Constants.WHEELS_TO_USSENSOR);
					nav.rotateByDeg(-90);
					
					//now again go until you've passed it and then a little extra and then aim your sensor forward again
					nav.goForward_imret();
					while (usp.getFilteredUSdistance() > (Constants.WHEELS_TO_BACK + Constants.WHEELS_TO_USSENSOR) * (2 - Constants.RELIABILITY_FACTOR)) {
						// don't check too often
						try{ Thread.sleep(Constants.RUNLOOP_PERIOD); }
						catch(InterruptedException e){e.printStackTrace();}	
					}
					while (usp.getFilteredUSdistance() < (Constants.WHEELS_TO_BACK + Constants.WHEELS_TO_USSENSOR) * (2 -Constants.RELIABILITY_FACTOR)) {
						//don't check too often
						try{ Thread.sleep(Constants.RUNLOOP_PERIOD); }
						catch(InterruptedException e){e.printStackTrace();}	
					}
					nav.stop();
					nav.forwardBy(Constants.WHEELS_TO_BACK + Constants.WHEELS_TO_USSENSOR);
					usp.rotateByDeg(90);
					
				//you should go around the obstacle to the left
				} else {
					//rotate to the left, aim sensor to the right and go until the obstacle is no longer to your right
					nav.rotateByDeg(-90);
					usp.rotateByDeg(90);
					nav.goForward_imret();
					
					//stop when the obstacle is no longer to your right
					while (usp.getFilteredUSdistance() < (Constants.EMERGENCY_DISTANCE) * (2 - Constants.RELIABILITY_FACTOR)) {
						//don't check too often
						try{ Thread.sleep(Constants.RUNLOOP_PERIOD); }
						catch(InterruptedException e){e.printStackTrace();}		
					}
					nav.stop();
					
					//then go some more so you can clear it when you turn back. turn back
					nav.forwardBy(Constants.WHEELS_TO_BACK + Constants.WHEELS_TO_USSENSOR);
					nav.rotateByDeg(90);
					
					//now again go until you've passed it and then a little extra and then aim your sensor forward again
					nav.goForward_imret();
					while (usp.getFilteredUSdistance() > (Constants.WHEELS_TO_BACK + Constants.WHEELS_TO_USSENSOR) * (2 - Constants.RELIABILITY_FACTOR)) {
						try{ Thread.sleep(Constants.RUNLOOP_PERIOD); }
						catch(InterruptedException e){e.printStackTrace();}		
					}
					while (usp.getFilteredUSdistance() < (Constants.WHEELS_TO_BACK + Constants.WHEELS_TO_USSENSOR) * (2 -Constants.RELIABILITY_FACTOR)) {
						//don't check too often
						try{ Thread.sleep(Constants.RUNLOOP_PERIOD); }
						catch(InterruptedException e){e.printStackTrace();}	
					}
					nav.stop();
					nav.forwardBy(Constants.WHEELS_TO_BACK + Constants.WHEELS_TO_USSENSOR);
					usp.rotateByDeg(-90);
					
					
				}
				
				//now that you avoided the obstacle, restart your travel to destination point from current location
				currState = State.TRAVELLING;
				homestretch = false;
				break;		
			}
		}
	}	

	/**
	 * determines if you should go around the obstacle by turning right or left and going by it
	 * @return true if you'll go to its right
	 */
	private static boolean shouldGoAroundObstacleByTurningRight() {
		if (!homestretch) {
			//go around towards your destination point
			if (((odo.getThetaDeg() % 360) + 360) % 360 < Math.PI * 3 / 4) {//if you're going rightwards
				return ydest < odo.getY();
			} else {
				return ydest > odo.getY();
			}
		} else {
			//turn away from the closest wall
			if (Constants.TILE_LENGTH * (Constants.PF_SIDE_LENGTH - 1) - odo.getX() > Constants.TILE_LENGTH + odo.getX()) { //if you're closer to the left wall
				return true;
			} else {
				return false;
			}
		}
	}
}
