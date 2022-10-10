package projet;

import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
 
public class UltrasonicSensor {
	
	private SensorModes ultrasonic;
	private SampleProvider distance;
	private float[] sample;
	
	public UltrasonicSensor(Port port) 
	{
		EV3UltrasonicSensor ultrasonic = new EV3UltrasonicSensor(port);
		this.distance= ultrasonic.getMode("Distance");
		this.sample = new float[distance.sampleSize()];
		
	}
	
	public void arrete() {
		 ((EV3UltrasonicSensor) ultrasonic).disable();
	}
	
	public boolean objectDetectedInRange(int range)
	{
		if(this.sample[0]<(range/100))
			return true;
		return false;
	}

	public boolean objectDetectedInRange()
	{
		if(this.sample[0]<0.5)
			return true;
		return false;
	}
	
	public float[] getSample() {
		return sample;
	}

	public float getSample0() {
		return sample[0];
	}
	
	public void setSample(float[] sample) {
		this.sample = sample;
	}
	
	public SensorModes getUltrasonic() {
		return ultrasonic;
	}

	public void setUltrasonic(SensorModes ultrasonic) {
		this.ultrasonic = ultrasonic;
	}

	public SampleProvider getDistance() {
		return distance;
	}

	public void setDistance(SampleProvider distance) {
		this.distance = distance;
	}
}