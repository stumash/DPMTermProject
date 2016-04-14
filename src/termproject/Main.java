package termproject;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.util.ArrayList;
import java.util.HashMap;

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
public class Main {
	//state-related variables
	private enum State {INIT, USLOCALIZE, LIGHTLOCALIZE, GOTOSTART,
		GOTOBALLS, PICKUPBALL, GOTONET, PREPARESHOT, FIRE, 
		DEFEND, TRAVELLING, OBSTACLEAVOIDANCE};
	private static State currState = State.INIT; //the current behavior state
	private static State nextState; //the previous behavior state
	
	//temp data for all the states
	private static double xdest, ydest;
	private static double filteredusD;
	private static double[] lightlocalangles = new double[4];
	private static double lightaverageangleone, lightaverageangletwo;
	private static double lighthalfanglediffone, lighthalfangledifftwo;
	private static double lightXoffset, lightYoffset, lightThetaOffsetX, lightThetaOffsetY;
	private static boolean homestretch, offense, ballsOnRight;
	private static ArrayList<MyPoint> dests;
	private static int destcounter = 0;
	private static int ballcounter = 0;

	//resources
	private static EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort(Constants.LEFT_MOTOR_PORT));
	private static EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort(Constants.RIGHT_MOTOR_PORT));
	private static EV3LargeRegulatedMotor sensorMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort(Constants.US_MOTOR_PORT));
	private static EV3LargeRegulatedMotor armMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort(Constants.ARM_MOTOR_PORT));
	
	//gameplay object instantiations
	private static Odometer odo = new Odometer(leftMotor, rightMotor);
	private static Timer odoTimer = new Timer(Constants.ODOMETER_PERIOD, odo);
	
	private static USPoller usp = new USPoller(LocalEV3.get().getPort(Constants.US_PORT), sensorMotor);
	private static Timer usTimer = new Timer(Constants.US_PERIOD, usp);
	
	private static LightPoller lp = new LightPoller(LocalEV3.get().getPort(Constants.LIGHT_PORT));
	private static Timer lightTimer = new Timer(Constants.LIGHT_PERIOD, lp);
	
	private static LCDinfo lcd = new LCDinfo(odo);
	private static Timer lcdTimer = new Timer(Constants.LCD_PERIOD, lcd);
	
	private static Navigator nav = new Navigator(leftMotor, rightMotor, odo);
	
	private static Localizer loc = new Localizer(nav, usp, odo);
	
	private static ArmManager arm = new ArmManager(armMotor);
	
	private static ColorPoller cp = new ColorPoller(LocalEV3.get().getPort(Constants.COLOR_PORT));
	
	//PrintWriter object for debugging
	private static PrintWriter writer;
	
	//main method
	public static void main(String[] args) throws FileNotFoundException, 
    UnsupportedEncodingException, InterruptedException{
		
		//start all timers
		odoTimer.start();
		usTimer.start();
		lightTimer.start();
		lcdTimer.start();
		
		//take a moment for all the timers to start up
		try{ Thread.sleep(1500); }
		catch(InterruptedException e){e.printStackTrace();}

		boolean dotheloop = true;		
///////////////////////////			random tester code here			/////////////////
//		dotheloop = false;
//		nav.rotateByDeg(360 * 3);
//		try{ Thread.sleep(5000); }
//		catch(InterruptedException e){e.printStackTrace();}
//		nav.rotateByDeg(-360 * 3);
////////////////////////// 		END OF TESTER CODE			////////////////////////		
		
		//the main run-loop
		runloop:
		while (dotheloop) {
			
			//run-loop period implemented
			try{ Thread.sleep(Constants.RUNLOOP_PERIOD); }
			catch(InterruptedException e){e.printStackTrace();}
			
			stateswitch:
			switch (currState) {
			
			/*
			 * The general setup state
			 */
			case INIT:
				nextState = State.USLOCALIZE;
				
				HashMap<String, Integer> setupData = WifiDPM.getWifiData();
				if (setupData == null) {
					System.exit(0);
				}
				if (setupData.get("OTN") == Constants.TEAM_NUMBER) {
					offense = true;
					loc.setCorner(StartCorner.lookupCorner(setupData.get("OSC")));
				} else {
					offense = false;
					loc.setCorner(StartCorner.lookupCorner(setupData.get("DSC")));
				}
				Constants.forward_line = (setupData.get("d2") - 1) * Constants.TILE_LENGTH;
				Constants.defender_line = (11 - setupData.get("d1")) * Constants.TILE_LENGTH;
				Constants.ll_x = setupData.get("ll-x") * Constants.TILE_LENGTH;
				Constants.ll_y = setupData.get("ll-y") * Constants.TILE_LENGTH;
				Constants.ur_x = setupData.get("ur-x") * Constants.TILE_LENGTH;
				Constants.ur_y = setupData.get("ur-y") * Constants.TILE_LENGTH;
				Constants.goal_width = setupData.get("w1");
			
				if (Math.abs(Constants.ur_x - Constants.TILE_LENGTH*11) < Math.abs(Constants.ll_x - Constants.TILE_LENGTH*(-1))) {
					/* if the upper-right corner of the ball-tray is closer to the right wall than the lower-left corner is
					 * to the left wall then the ball-tray is on the right side of the field and the tray's left hand side
					 * should be approached from the left for ball collection
					 */
					Constants.balldest_x = Constants.ll_x - Constants.TILE_LENGTH;
					Constants.balldest_y = Constants.ll_y - Constants.TILE_LENGTH;
					ballsOnRight = true;
				} else {
					Constants.balldest_x = Constants.ur_x + Constants.TILE_LENGTH;
					Constants.balldest_y = Constants.ur_y + Constants.TILE_LENGTH;
					ballsOnRight = false;
				}
				
				if (offense) {
					Constants.startx = Constants.TILE_LENGTH * 5; //middle of playing field
					Constants.starty = Constants.forward_line - Constants.TILE_LENGTH;
				} else {
					Constants.startx = Constants.TILE_LENGTH * 5; //middle of playing field
					Constants.starty = Constants.defender_line + Constants.TILE_LENGTH;
				}

				
				Constants.firedest_x = (Constants.TILE_LENGTH * 5) + (Constants.goal_width/2);
				Constants.firedest_y = Constants.forward_line - (Constants.TILE_LENGTH / 2);
				Constants.firedest_th = -(Constants.FIRE_ERROR_OFFSET_ANGLE + 90);
				
				try { Thread.sleep(1500); }
				catch (InterruptedException e) { e.printStackTrace(); }
				currState = nextState;
				break stateswitch;
				
			/*
			 * This state state is for US localization, which is the first action performed by the robot.  The US localization routine
			 * allows the robot to know its initial position and heading relative to some point and heading (0,0,0).
			 */
			case USLOCALIZE:
				nextState = State.GOTOSTART;
				
				loc.localize();
				Sound.beep();
				
				currState = nextState;
				break stateswitch;
							
				
			/*
			 * This state is entered to perform light-localization while away from a corner. The method relies on
			 * the robot being in a particular zone relative to a known intersection of ground lines on the field. 
			 */
			case LIGHTLOCALIZE:
				
				for (int loccount = 0; loccount < 2; loccount++) {//do this whole thing twice
					
				//attempt to correct position before continuing, if necessary
				if (!nav.isWithinDistOfNearestIntersection(2)) {
					double[] nearest = nav.nearestIntersectionCoords();
					nav.rotateToXdest(nearest[0]);
					nav.forwardBy(Math.abs(nearest[0] - odo.getX()));
					nav.rotateToYdest(nearest[1]);
					nav.forwardBy(Math.abs(nearest[1] - odo.getY()));
				}
					
				//face the robot currently thinks is forward
				nav.rotateToDeg(0);
				
				//begin rotating full circle
				nav.rotateByDeg_imret(360);			
				for (int i = 0; i < 4; i++) {
					if (i == 0) {
						try { Thread.sleep(1000); }
						catch (InterruptedException e) { e.printStackTrace(); }
					}
					//wait to be looking at a line
					while (lp.getLight() > Constants.LIGHT_THRESHOLD) {
						//don't check too often
						try { Thread.sleep(Constants.RUNLOOP_PERIOD); }
						catch (InterruptedException e) { e.printStackTrace(); }
					}
					
					//now that you're looking at a line, record the angle
					Sound.beep();
					lightlocalangles[i] = odo.getThetaDeg() 
							- Constants.ROTATE_SPEED * Constants.RUNLOOP_PERIOD / 1000;//angle travelled while waiting for odo.getTheta()			
					//wait to not be looking at a line anymore
					while (lp.getLight() < Constants.LIGHT_THRESHOLD) {
						//don't check too often
						try { Thread.sleep(Constants.RUNLOOP_PERIOD); }
						catch (InterruptedException e) { e.printStackTrace(); }
					}					

					//now do this for all four points
				}
				
				//wait to finish the rotation + data collection
				while (nav.isMoving()) {
					//don't check too often
					try { Thread.sleep(Constants.RUNLOOP_PERIOD); }
					catch (InterruptedException e) { e.printStackTrace(); }
				}
				
				//record (first angle - third angle) and (fourth angle - second angle). convert the diffs to [0,360] 
				lighthalfanglediffone = 0.5 * on0to360(lightlocalangles[0] - lightlocalangles[2]); 
				lighthalfangledifftwo = 0.5 * on0to360(lightlocalangles[3] - lightlocalangles[1]);
						
				//record the average of: first and third angle, second angle and fourth angle 
				lightaverageangleone = on0to360(lightlocalangles[2] + lighthalfanglediffone);	//ideally = -Constants.LIGHTSENSOR_ANGLE_OFFSET		
				lightaverageangletwo = on0to360(lightlocalangles[1] + lighthalfangledifftwo);	//ideally = -Constants.LIGHTSENSOR_ANGLE_OFFSET - 90
								
				//calculate the error in the x and y direction												
				lightXoffset = Math.cos(Math.toRadians(lighthalfanglediffone)) * Constants.WHEELCENTER_TO_LIGHTSENSOR; 		
				lightYoffset = Math.cos(Math.toRadians(lighthalfangledifftwo)) * Constants.WHEELCENTER_TO_LIGHTSENSOR;		
				
				//calculate the average angle error
				lightThetaOffsetX = (-Constants.LIGHTSENSOR_ANGLE_OFFSET - lightaverageangleone) % 360;
				lightThetaOffsetY = ((-Constants.LIGHTSENSOR_ANGLE_OFFSET - 90) - lightaverageangletwo) % 360;	
								
				//update odometer with correction
				odo.setPositionDeg(new double[] {odo.getX() - lightXoffset, odo.getY() - lightYoffset, odo.getThetaDeg() + 0.5 * (lightThetaOffsetX + lightThetaOffsetY)});
				
				}//end of for-loop
				
				nav.rotateToDeg(0);
				currState = nextState;
				break stateswitch;
				
				
			/*
			 * This state is for setting the start coordinates and getting to them in steps.
			 */
			case GOTOSTART:	
				if (dests == null) {
					nextState = currState;
					dests = makeDestlistFromDest(Constants.startx, Constants.starty);
				}
				if (dests != null) {
					if (destcounter < dests.size()) {
						xdest = dests.get(destcounter).getX();
						ydest = dests.get(destcounter).getY();
						currState = State.TRAVELLING;
						destcounter++;
					} else {
						destcounter = 0;
						if (offense) {
							nextState = State.GOTOBALLS;
						} else {
							nextState = State.DEFEND;
						}
						dests = null;
					}
				}
					
				break;
				
			/*
			 * This state includes all the behaviour of the robot while defends the net, remaining in the
			 * defensive zone
			 */
			case DEFEND:
				//no defense strategy, just sit there
				break runloop;
				
				
			/*
			 * This state includes all the behaviour of the in determining how to get to a ball
			 */
			case GOTOBALLS:
				if (dests == null) {
					nextState = currState;
					dests = makeDestlistFromDest(Constants.balldest_x, Constants.balldest_x);
				}
				if (dests != null) {
					if (destcounter < dests.size()) {
						xdest = dests.get(destcounter).getX();
						ydest = dests.get(destcounter).getY();
						currState = State.TRAVELLING;
						destcounter++;
					} else {
						destcounter = 0;
						nextState = State.PICKUPBALL;
						dests = null;
					}
				}
				
				currState = nextState;
				break stateswitch;
				
			/*
			 * this state includes all the robot's "detect and pick up a ball" behaviour
			 */
			case PICKUPBALL:
				nextState = State.GOTONET;
				
				//allign with balls
				if (ballsOnRight) {
					nav.rotateToDeg_imret(90);
					nav.forwardBy(Constants.TILE_LENGTH - Constants.WHEELCENTER_TO_LIGHTSENSOR);
					nav.rotateByDeg(-90);
				} else {
					nav.rotateToDeg_imret(-90);
					nav.forwardBy(Constants.TILE_LENGTH - Constants.WHEELCENTER_TO_LIGHTSENSOR);
					nav.rotateByDeg(90);
				}
				
				//go as far as the current one
				nav.forwardBy(Constants.INCH * 1.5);
				for (int i = 0; i < ballcounter; i++) {
					nav.forwardBy(Constants.INCH * 3);
				}
				ballcounter++; //next time get the next ball
				
				//pick it up
				arm.collectBall();
				
				currState = nextState;
				break;
				
			/*
			 * This state is for getting to the net
			 */
			case GOTONET:
				
				if (dests == null) {
					nextState = currState;
					dests = makeDestlistFromDest(Constants.firedest_x, Constants.firedest_y);
				}
				if (dests != null) {
					if (destcounter < dests.size()) {
						xdest = dests.get(destcounter).getX();
						ydest = dests.get(destcounter).getY();
						currState = State.TRAVELLING;
						destcounter++;
					} else {
						destcounter = 0;
						nextState = State.PREPARESHOT;
						dests = null;
					}
				}
				
				currState = nextState;
				break;
			case PREPARESHOT:
				nextState = State.FIRE;
				
				nav.rotateToDeg(Constants.firedest_th);
				
				currState = nextState;
				break;	
			case FIRE:
				nextState = State.GOTOBALLS;
				
				arm.fire();
				
				currState = nextState;
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
					currState = State.LIGHTLOCALIZE;					
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
					nav.forwardBy(Constants.WHEELS_TO_BACK + Constants.WHEELS_TO_USSENSOR + 5);
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
					nav.forwardBy(Constants.WHEELS_TO_BACK + Constants.WHEELS_TO_USSENSOR + 5);
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
			} //end of switchstate
		} //end of runloop
		
		while(Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	} //end of main method

	/**
	 * determines if you should go around the obstacle by turning right or left and going by it
	 * @return true if you'll go to its right
	 */
	private static boolean shouldGoAroundObstacleByTurningRight() {
		if (!homestretch) {
			//go around towards your destination point
			if (((odo.getThetaDeg() % 360) + 360) % 360 < 135) {//if you're going rightwards
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
	
	/**
	 * takes an angle in degrees on [-inf,inf] and returns the same angle in degrees on [0, 360]
	 * @param deg the angle, in degrees, that you wish to convert to the interval [0,360]
	 * @returns the angle passed argument converted to the interval [0,360]
	 */
	private static double on0to360(double deg) {
		double result = deg % 360;
		if (result < 0) {
			result += 360;
		}
		return result;
	}
	
	public static ArrayList<MyPoint> makeDestlistFromDest(double x, double y) {
		ArrayList<MyPoint> result = new ArrayList<MyPoint>();
		result.add(0, new MyPoint(x, y));
		double x0 = 0; double y0 = 0;
		if (Math.abs(x - odo.getX()) > Constants.TILE_LENGTH * 5.1 || Math.abs(y - odo.getY()) > Constants.TILE_LENGTH * 5.1) {
			if(Math.abs(x - odo.getX()) > Constants.TILE_LENGTH * 5.1) {
				if (x > odo.getX()) {
					x0 = odo.getX() + Constants.TILE_LENGTH * 5;
				} else {
					x0 = odo.getX() - Constants.TILE_LENGTH * 5;
				}
			} else {
				x0 = x;
			}
			if(Math.abs(y - odo.getY()) > Constants.TILE_LENGTH * 5.1) {
				if (y > odo.getY()) {
					y0 = odo.getY() + Constants.TILE_LENGTH * 5;
				} else {
					y0 = odo.getY() - Constants.TILE_LENGTH * 5;
				}
			} else {
				y0 = y;
			}
			result.add(0,new MyPoint(x0 + odo.getX(), y0 + odo.getY()));
		}
		return result;
	}

}
