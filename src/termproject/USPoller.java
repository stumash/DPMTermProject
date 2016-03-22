package termproject;

import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.TimerListener;

/**
 * a class used to abstract away the details of the ultrasonic
 * sensor in favor of an API that simplifies the ultrasonic sensor's usage down to a single getter: getUSDistance.
 * @author Stuart Mashaal and Mathieu Savoie
 *
 */
public class USPoller implements TimerListener{
	//sensor-related resources
	Port port;
	EV3UltrasonicSensor sensor = new EV3UltrasonicSensor(port);
	SampleProvider sp = sensor.getDistanceMode();
	float[] samples = new float[sensor.sampleSize()];
	
	/**
	 * constructs an instance of USPoller
	 * @param p the EV3 port that the US sensor is connected to
	 */
	public USPoller(Port p) {
		this.port = p;
	}
	
	@Override
	public void timedOut() {
		synchronized (this) {
			sp.fetchSample(samples, 0);
		}
	}
	
	/**
	 * 
	 * @return the current distance measurement taken by the ultrasonic sensor
	 */
	public float getUSdistance() {
		synchronized (this) {
			return samples[0];
		}
	}
	
}