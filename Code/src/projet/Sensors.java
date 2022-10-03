package projet;

import java.util.Arrays;
import lejos.robotics.filter.MeanFilter;
import lejos.utility.Delay;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.Color;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.hardware.sensor.EV3TouchSensor;

public class Sensors {

	private EV3TouchSensor touchSensor;
	private EV3UltrasonicSensor ultrasonicSensor;
	//private EV3ColorSensor colorSensor;
	private static SampleProvider spU;
	private static SampleProvider spT;
	/*public static String BLACK;
	 * public static String YELLOW;
	 * public static String GREEN;
	 * public static String RED;
	 * public static String BLUE
	 * public static String WHITE;
	 * public static String GRAY;*/
	
	//Cr�er methode getcolor 
	
	public Sensors(Port ultrasonicPort,Port touchPort) {
		// TODO Auto-generated constructor stub
		ultrasonicSensor = new EV3UltrasonicSensor(ultrasonicPort);
		spU= ultrasonicSensor.getMode("Distance");
		touchSensor= new EV3TouchSensor(touchPort);
		spT= touchSensor.getTouchMode();
		//rajouter param�tre colorsensor et l'initialiser ainsi que les valeur des couleur

	}
	
	public EV3TouchSensor getTouchSensor(){
		return touchSensor;
	}
	
	public EV3UltrasonicSensor getUltrasonicSensor() {
		return ultrasonicSensor;
	}
	
	
	public void setTouchSensor(EV3TouchSensor t) {
		touchSensor=t;
	}
	
	public void setUltrasonicSensor(EV3UltrasonicSensor u) {
		ultrasonicSensor=u;
	}
	
	public float getDistance() {
		float[] sample=new float[spU.sampleSize()];
		spU.fetchSample(sample, 0);
		return sample[0];
	}
	
	public int getDistanceInt() {
		return (int)getDistance();
	}
	
	public float getDistanceMean() {
		float[] tab=new float[10];
		for(int i=0;i<10;i++) {
			tab[i]=getDistance();
		}
		Arrays.sort(tab);
		float res=0;
		for(int i=2;i<8;i++) {
			res+=tab[i];
		}
		return res/(float)6;
	}
	
	public int getDistanceMeanInt() {
		int[] tab=new int[10];
		for(int i=0;i<10;i++) {
			tab[i]=getDistanceInt();
		}
		Arrays.sort(tab);
		int res=0;
		for(int i=2;i<8;i++) {
			res+=tab[i];
		}
		return res/6;
	}
	
	public boolean Touch() {
		float[] sample=new float[spT.sampleSize()];
		spT.fetchSample(sample, 0);
		return sample[0]!=(float)0;
	}
	
	public boolean objectDetectedInRange(int range){
	return this.getDistance()<((float)range/(float)100);
	}

	public boolean objectDetectedInRange(float range){
		return this.getDistance()<range;
		}
	
	public boolean objectDetectedInRange(){
		return objectDetectedInRange((float)0.5);
	}
	

	public static void main(String[] args) {
		// TODO Auto-generated method stub

		
	}

}
