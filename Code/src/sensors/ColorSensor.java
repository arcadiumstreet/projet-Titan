package sensors;

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

	/**
	 * Initiallise une instance de ColorSensor en attribuant :
	 * à l'attribut port_Color le port correspondant à la chaine de caractere en parametre;
	 * à l'attribut sensor_Color l'EV3ColorSensor branché sur le port port_Color, mettre Mode courant sur "RGB" et la Floodlight sur couleur Blanche
	 * 
	 * @param port la chaine de caractere représentant le port de branchement de l'EV3ColorSensor
	 */
	public ColorSensor(String port) {
		port_Color = LocalEV3.get().getPort(port);
		sensor_Color = new EV3ColorSensor(port_Color);
		sensor_Color.setCurrentMode("RGB");
		sensor_Color.setFloodlight(Color.WHITE);
	}

	/**
	 * 
	 * @return la couleur detectee par le capteur
	 */
	public static Color getColor(){	
		sample = new float[sensor_Color.sampleSize()];
		sensor_Color.fetchSample(sample, 0);
		return new Color((int)(sample[0] * 255), (int)(sample[1] * 255), (int)(sample[2] * 255));
	}

	/**
	 * 
	 * @return un tableau d'entier contenant les valeurs "RGB" de la couleur captée par le ColorSensor
	 */
	public int[] getcolorint() {
		sample = new float[sensor_Color.sampleSize()];
		sensor_Color.fetchSample(sample, 0);
		return new int [] {(int)(sample[0] * 255), (int)(sample[1] * 255), (int)(sample[2] * 255)};
    }
	
	/**
	 * donne la chaine de caractere de la couleur decrite par les valeur r, g, b.
	 * 
	 * @param r la portion de rouge dans la couleur décrite
	 * @param g la portion de vert dans la couleur décrite
	 * @param b la portion de bleu dans la couleur décrite
	 * @return une chaine de caractere dans les suivantes : "NOIR","GRIS","JAUNE","BLANC","ROUGE","VERT","BLEU" ou "NON RECONNU" si les valeurs de r, g et b ne correspondent à auccune couleur predefinie
	 */
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