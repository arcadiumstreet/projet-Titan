package sensors;

import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
 
public class UltrasonicSensor {
	
	private EV3UltrasonicSensor ultrasonic;
	private SampleProvider distance;
	private float[] sample;
	
	/**
	 * 
	 * @param port
	 */
	public UltrasonicSensor(Port port) 
	{
		 ultrasonic = new EV3UltrasonicSensor(port);
		this.distance= ultrasonic.getMode("Distance");
		this.sample = new float[distance.sampleSize()];
		
	}
	
	/**
	 * 
	 */
	public void stop() {
		ultrasonic.disable();
	}
	
	/**
	 * 
	 * @param range
	 * @return
	 */
	public boolean objectDetectedInRange(int range)
	{
		if(this.sample[0]<(range/100))
			return true;
		return false;
	}

	/**
	 * 
	 * @return
	 */
	public boolean objectDetectedInRange()
	{
		if(this.sample[0]<0.5)
			return true;
		return false;
	}
	
	/**
	 * 
	 * @return
	 */
	public float[] getSample() {
		return sample;
	}

	/**
	 * 
	 * @return
	 */
	public float getSample0() {
		return sample[0];
	}
	
	/**
	 * 
	 * @param sample
	 */
	public void setSample(float[] sample) {
		this.sample = sample;
	}
	
	/**
	 * 
	 * @return
	 */
	public SensorModes getUltrasonic() {
		return ultrasonic;
	}

	/**
	 * 
	 * @param ultrasonic
	 */
	public void setUltrasonic(EV3UltrasonicSensor ultrasonic) {
		this.ultrasonic = ultrasonic;
	}

	/**
	 * 
	 * @return
	 */
	public SampleProvider getDistance() {
		return distance;
	}

	/**
	 * 
	 * @param distance
	 */
	public void setDistance(SampleProvider distance) {
		this.distance = distance;
	}
}