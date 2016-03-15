package termproject;

/**
 * a class used to abstract away the details of the color sensor in favor of an API that
 * simplifies the color sensor's usage down to a single getter method: getColor().
 * @author Stuart Mashaal and Mathieu Savoie
 *
 */
public class ColorPoller {

	/**
	 * constructs an instance of ColorPoller
	 */
	public ColorPoller() {
		
	}
	/**
	 * 
	 * @return the current color value detected by the color sensor
	 */
	public double getColor() {
		return 0;
	}
}
