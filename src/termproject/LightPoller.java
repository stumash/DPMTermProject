package termproject;

import lejos.utility.TimerListener;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * a class used to abstract away the details of the color sensor in favor of an API that
 * simplifies the color sensor's usage down to a single getter method: getColor().
 * @author Stuart Mashaal
 *
 */
public class LightPoller implements TimerListener{
	//sensor-related resources
	Port port;
	SensorModes sensor;
	SampleProvider sp;
	float[] samples;
		
	/**
	 * constructs an instance of ColorPoller
	 */
	public LightPoller(Port p) {
		this.port = p;
		sensor = new EV3ColorSensor(port);
		sp = sensor.getMode("Red");
		samples = new float[sensor.sampleSize()];;
	}
	
	public void timedOut() {
		synchronized (this) {
			sp.fetchSample(samples, 0);
		}
	}
	
	/**
	 * 
	 * @return the current color value detected by the color sensor
	 */
	public double getLight() {
		synchronized (this) {
			return samples[0] * 100;
		}
	}
}
