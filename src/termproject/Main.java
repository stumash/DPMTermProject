package termproject;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;

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
	private static boolean homestretch;
	private static int counter = 0;

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
		writer = new PrintWriter("lightlocal.csv", "UTF-8");
		
		//start all timers
		odoTimer.start();
		usTimer.start();
		lightTimer.start();
		lcdTimer.start();
		
		//take a moment for all the timers to start up
		try{ Thread.sleep(1500); }
		catch(InterruptedException e){e.printStackTrace();}

		boolean dotheloop = true;
		////////////			random tester code here			/////////////////
//		dotheloop = false;
//		
//		//navigation test
//		nav.forwardBy(Constants.TILE_LENGTH * 5);
//		//nav.rotateByDeg(720);
		
		
		
		//the main run-loop
		runloop:
		while (dotheloop) {
			
			//run-loop period implemented
			try{ Thread.sleep(Constants.RUNLOOP_PERIOD); }
			catch(InterruptedException e){e.printStackTrace();}
			
			stateswitch:
			switch (currState) {
			case INIT:
				nextState = State.USLOCALIZE;
				
				//TODO create the value-setting logic for this using wifi classes
				
				loc.setCorner(1);
				
				currState = nextState;
				break;
				
			/*
			 * This state state is for US localization, which is the first action performed by the robot.  The US localization routine
			 * allows the robot to know its initial position and heading relative to some point and heading (0,0,0).
			 */
			case USLOCALIZE:
				nextState = State.GOTOSTART;
				
				//perform localization
				loc.localize();
				
				currState = nextState;
				break;
							
				
			/*
			 * This state is entered to perfrom light-localization while away from a corner. The method relies on
			 * the robot being in a particular zone relative to a known intersection of ground lines on the field. 
			 */
			case LIGHTLOCALIZE:
				
				//face forward
				nav.rotateToDeg(0);
				
				//begin rotating full circle
				nav.rotateByDeg_imret(360);			
				for (int i = 0; i < 4; i++) {
					
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
				
				writer.write("diffone: " + lighthalfanglediffone + "\ndifftwo: " + lighthalfangledifftwo);
				writer.write("\naverageone: " + lightaverageangleone + "\naveragetwo: " + lightaverageangletwo);
				writer.flush();
				
				currState = State.GOTOSTART;
				break;
				
			/*
			 * This state is for setting the start coordinates.
			 */
			case GOTOSTART:
				
				if (counter == 0) {
					counter++;
					xdest = (Constants.TILE_LENGTH * 5);
					ydest = (Constants.TILE_LENGTH * 5);
					nextState = State.LIGHTLOCALIZE;
				} else {
					counter++;
					xdest = 0;
					ydest = 0;
					nextState = State.DEFEND;
				}
				
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
//					
//					//now again go until you've passed it and then a little extra and then aim your sensor forward again
//					nav.goForward_imret();
//					while (usp.getFilteredUSdistance() > (Constants.WHEELS_TO_BACK + Constants.WHEELS_TO_USSENSOR) * (2 - Constants.RELIABILITY_FACTOR)) {
//						try{ Thread.sleep(Constants.RUNLOOP_PERIOD); }
//						catch(InterruptedException e){e.printStackTrace();}		
//					}
//					while (usp.getFilteredUSdistance() < (Constants.WHEELS_TO_BACK + Constants.WHEELS_TO_USSENSOR) * (2 -Constants.RELIABILITY_FACTOR)) {
//						//don't check too often
//						try{ Thread.sleep(Constants.RUNLOOP_PERIOD); }
//						catch(InterruptedException e){e.printStackTrace();}	
//					}
//					nav.stop();
//					nav.forwardBy(Constants.WHEELS_TO_BACK + Constants.WHEELS_TO_USSENSOR);
					nav.forwardBy(Constants.TILE_LENGTH * 1.5);
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

}
