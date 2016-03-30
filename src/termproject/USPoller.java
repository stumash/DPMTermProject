package termproject;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
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
	Port port;
	EV3UltrasonicSensor sensor;
	SampleProvider sp;
	float[] samples;
	EV3LargeRegulatedMotor sensorMotor;
	
	/**
	 * constructs an instance of USPoller
	 * @param p the EV3 port that the US sensor is connected to
	 */
	public USPoller(Port p, EV3LargeRegulatedMotor sensorMotor) {
		this.port = p;
		sensor = new EV3UltrasonicSensor(port);
		sp = sensor.getDistanceMode();
		samples = new float[sensor.sampleSize()];
		
		this.sensorMotor = sensorMotor;
	}

	public void timedOut() {
		synchronized (this) {
			sp.fetchSample(samples, 0);
		}
	}

//data retrieval methods
	/**
	 * 
	 * @return the current distance measurement taken by the ultrasonic sensor
	 */
	public double getUSdistance() {
		synchronized (this) {
			return samples[0] * 100;
		}
	}
	/**
	 * 
	 * @return the value measured by the US sensor filtered out below Constant.MAX_US_FILTER 
	 */
	public double getFilteredUSdistance() {
		double result = getUSdistance();
		if (result > Constants.MAX_US_FILTER) {
			result = Constants.MAX_US_FILTER;
		}
		return result;	
	}

//sensorMotor helper methods
	/**
	 * rotate the sensor to aim it in another direction
	 * @param theta the angle, in degrees, by which to rotate the sensor
	 */
	public void rotateByDeg(double theta) {
		sensorMotor.rotate((int)theta);
	}
	/**
	 * rotate the sensor to aim it in another direction.  Method call returns immediately, regardless
	 * of whether the motion is completed yet
	 * @param theta the angle, in degrees, by which to rotate the sensor
	 */
	public void rotateByDeg_imret(double theta) {
		sensorMotor.rotate((int)theta,true);
	}
	/**
	 * rotate the sensor to aim it in another direction
	 * @param theta the angle, in radians, by which to rotate the sensor
	 */
	public void rotateByRad(double theta) {
		rotateByDeg((int)Math.toDegrees(theta));
	}
	/**
	 * rotate the sensor to aim it in another direction
	 * @param theta the angle, in radians, by which to rotate the sensor
	 */
	public void rotateByRad_imret(double theta) {
		rotateByDeg_imret(Math.toDegrees(theta));
	}
	
	
}