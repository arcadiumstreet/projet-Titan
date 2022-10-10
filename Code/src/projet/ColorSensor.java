package projet;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.BaseSensor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;


public class ColorSensor  {
	
	private static SensorModes color ;
	private SampleProvider d;
	
	public ColorSensor(Port port){
		EV3ColorSensor color=new EV3ColorSensor(port);
		}
	
	public Color getColorOnGround() {
		color.setCurrentMode("RGB");
		float[] sample = new float[d.sampleSize()];
		color.fetchSample(sample, 0);
		return new Color((int)(sample[0] * 255), (int)(sample[0] * 255), (int)(sample[0] * 255));
	}
	
public String toString(int r, int g, int b) {
		
		if((r<6 && g<6 && b<6)) {
			return "NOIR";
		}else if((r>8 && g>8 && b>8) && (r<27 && g<27 && b<27)) {
			return "GRIS";
		}else if((r>32 && g>22 && b<12)) {
			return "JAUNE";
		}else if(r>25 && g>25 && b>25) {
			return "BLANC";
		}else if(r>g && r>b) {
			return "ROUGE";
		}else if(g>r && g>b) {
			return "VERT";
		}else if(b>r && b>g) {
			return "BLEU";
		}else {
			return "NON RECONNU";
		}
	
	
	
	}}