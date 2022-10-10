package projet;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.BaseSensor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;


public class ColorSensor  {
	
	private Port port_Color;
	private static EV3ColorSensor sensor_Color;
	public static float[] sample;

	public ColorSensor(String port) {
		port_Color = LocalEV3.get().getPort(port);
		sensor_Color = new EV3ColorSensor(port_Color);
		sensor_Color.getRGBMode();
		sensor_Color.setFloodlight(Color.WHITE);
	}

	public static Color getColorIn(){	
		sample = new float[sensor_Color.sampleSize()];
		sensor_Color.fetchSample(sample, 0);
		return new Color((int)(sample[0] * 255), (int)(sample[0] * 255), (int)(sample[0] * 255));
	}

	public int[] colorint() {
		sample = new float[sensor_Color.sampleSize()];
		sensor_Color.fetchSample(sample, 0);
		return new int [] {(int)(sample[0] * 255), (int)(sample[0] * 255), (int)(sample[0] * 255)};
                //getColor().toString(getColor().getRed(),getColor().getGreen(),getColor().getColor().getBlue());
    }
	
	public static String color_String(int r, int g, int b) {
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