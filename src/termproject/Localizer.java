package termproject;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class is for getting the robot to figure out where it is.  It
 * has only one method, 'localize'.
 * @author Stuart Mashaal
 *
 */
public class Localizer {
	private Navigator nav;
	private USPoller usp;
	private Odometer odo;
	
	//to contain the post-localization odometer settings, which differ based on the corner the robot is first set down in
	private double myCornerCoords[] = new double[3];

	/**
	 * constructs a Localizer instance with referenced to to wheel motors (left and right),
	 * an UltraSonic Poller and a Color Poller.
	 * @param leftMotor the robot's left motor
	 * @param rightMotor the robot's right motor
	 * @param usp the ultrasonic poller for distance-to-wall detection
	 * @param cp the color poller for line detection
	 */
	public Localizer(Navigator nav, USPoller usp, Odometer odo) {
		this.nav = nav;
		this.usp = usp;
		this.odo = odo;
	}
	
	/**
	 * executes localization routine
	 */
	public void localize() {
		//get yourself to face away from the wall
		double angleofMindist = odo.getThetaDeg();
		double mindist = Constants.MAX_US_FILTER;
		nav.rotateByDeg_imret(360);
		while(nav.isMoving()) {
			if (usp.getFilteredUSdistance() < mindist) {
				mindist = usp.getFilteredUSdistance();
				angleofMindist = odo.getThetaDeg();
			}
		}
		nav.rotateToDeg(angleofMindist + 180);
		usp.rotateByDeg(90);
		if (usp.getFilteredUSdistance() < Constants.MAX_US_FILTER) { //if theres a wall to your right
			nav.rotateByDeg(-90);
		}
		usp.rotateByDeg(-90); //move the US sensor back to straight ahead
		
		//now that you're facing away from the wall, do the following to get to the myCornerCoords
		nav.localizerBackup();
		nav.forwardBy(Constants.TILE_LENGTH - Constants.WHEELS_TO_BACK);
		nav.rotateByDeg(90);
		nav.localizerBackup();
		nav.forwardBy(Constants.TILE_LENGTH - Constants.WHEELS_TO_BACK);
		nav.rotateByDeg(-90);
		
		//now that you're at the myCornerCoords, update Odometer
		odo.setPosition(myCornerCoords);
	}
	
	/**
	 *TODO
	 * @return
	 */
	public double[] getMyCornerCoords() {
		return myCornerCoords;
	}

	public void setMyCornerCoords(double[] xytheta) {
		this.myCornerCoords = xytheta;
	}
}
