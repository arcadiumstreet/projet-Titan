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
	 * initiallise une instance de UltrasonicSensor en attribuant : 
	 * à l'attribut ultrsonic l'EV3UltrasonicSensor branché sur le port passe en parametre,
	 * à l'attribut distance le SampleProvider du Mode "Distance" de l'ultrasonicSensor,
	 * à l'attribut sample un tableau de float de la taille des echantillons donne par le SmpleProvider distance
	 * @param port le port de branchement de l'EV3UltrasonicSensor
	 */
	public UltrasonicSensor(Port port) 
	{
		 ultrasonic = new EV3UltrasonicSensor(port);
		this.distance= ultrasonic.getMode("Distance");
		this.sample = new float[distance.sampleSize()];	
	}
	
	/**
	 * Eteind le capteur ultrasonic
	 */
	public void stop() {
		ultrasonic.disable();
	}
	
	/**
	 * Detecte si un objet se situe à une distance inférieur a la range(en metre) du capteur
	 * @param range en metres
	 * @return true si object detecte à moins de range metres, false sinon
	 */
	public boolean objectDetectedInRange(int range)
	{
		if(this.sample[0]<(range/100))
			return true;
		return false;
	}

	/**
	 * Detecte si un objet se situe à une distance inférieur a la 50cm du capteur
	 * @return true si le capteur detecte quelque chose à moins de 50cm, false sinon
	 */
	public boolean objectDetectedInRange()
	{
		if(this.sample[0]<0.5)
			return true;
		return false;
	}
	
	/**
	 * @return un tableau de float contenant les perception du capteur ultrasonic
	 */
	public float[] getSample() {
		return sample;
	}

	/**
	 * donne la distance à laquelle le capteur detecte un objet
	 * @return la distance(en metre) à laquelle le capteur detecte quelque chose ou INFINITY si rien n'est détecté sous la portée maximal du capteur
	 */
	public float getSample0() {
		return sample[0];
	}
	
	/**
	 * affect à l'attribut this.sample le tableau de float sample
	 * @param sample
	 */
	public void setSample(float[] sample) {
		this.sample = sample;
	}
	
	/**
	 * @return ultrasonic
	 */
	public SensorModes getUltrasonic() {
		return ultrasonic;
	}

	/**
	 * affect à l'attribut this.ultrasonic l'EV3UltrasonicSensor ultrasonic
	 * @param ultrasonic
	 */
	public void setUltrasonic(EV3UltrasonicSensor ultrasonic) {
		this.ultrasonic = ultrasonic;
	}

	/**
	 * @return le SampleProvider distance
	 */
	public SampleProvider getDistance() {
		return distance;
	}

	/**
	 * affect à l'attribut this.distance le SampleProvider distance
	 * @param distance
	 */
	public void setDistance(SampleProvider distance) {
		this.distance = distance;
	}
}