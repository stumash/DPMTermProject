package termproject;

/**
 * a class used to abstract away the details of the ultrasonic
 * sensor in favor of an API that simplifies the ultrasonic sensor's usage down to a single getter: getUSDistance.
 * @author Stuart Mashaal and Mathieu Savoie
 *
 */
public class USPoller {
	
	/**
	 * constructs an instance of USPoller
	 */
	public USPoller() {
		
	}
	
	/**
	 * 
	 * @return the current distance measurement taken by the ultrasonic sensor
	 */
	public double getUSdistance() {
		return 0;
	}
	
}
