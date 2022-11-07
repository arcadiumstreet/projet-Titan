package projet;

import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.motor.MindsensorsGlideWheelMRegulatedMotor;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.EncoderMotor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import java.math.*;
import lejos.hardware.sensor.EV3TouchSensor;


public class FirstClass {
	
	static Robot pierrot = new Robot(MotorPort.B,MotorPort.C,MotorPort.A,SensorPort.S4,SensorPort.S3,SensorPort.S1);

	
	/**
	 * methode strategie1 qui est appelee au depart du round lorsqu tous les palets sont la
	 * @param d va prendre 45 ou -45 en fonction de l'endroit de depart 
	 * @param d2 va prendre 45 ou -45 en fonction de l'endroit de depart 
	 * @param placement va prendre 0,1,2 en fonction de si il est a gauche au mileu ou a droite 
	 * @param angle angle vers lequel s'orienter pour trouver le 2 eme palet (155 ou -155)
	 */
	public static void strategie1(int d, int d2, int placement, double angle) {}
	
	/**
	 * methode strategie1 qui est appelee apres la pause du round lorsqu'au moins encore un palet est bien plac�
	 * @param d d va prendre 45 ou -45 en fonction de l'endroit de depart 
	 * @param d2 va prendre 45 ou -45 en fonction de l'endroit de depart
	 * @param placement va prendre 0,1,2 en fonction de si il est a gauche au mileu ou a droite 
	 * @param angle angle vers lequel s'orienter pour trouver le 2 eme palet (155 ou -155)
	 */
	public static void strategie2(int d, int d2, int placement, double angle) {}
	
	
	
	public static void main(String[] args) {
		
		

		  
	
		
		/*pierrot.getUltrasonics().getDistance().fetchSample(pierrot.getUltrasonics().getSample(), 0);
		float dist = pierrot.getUltrasonics().getSample()[0];
		pierrot.getRightGear().rotate(360, true);
		int i = 0;
		while (Math.abs(pierrot.getUltrasonics().getSample()[0] - dist) < 0.1 && i < 30) {
			dist = pierrot.getUltrasonics().getSample()[0];
			
			i++;
		}
		pierrot.getRightGear().stop();
		if(i<30)
			pierrot.catchTarget((int) (pierrot.getUltrasonics().getSample()[0]));*/

		//pierrot.moveCm(pierrot.FRONT, 100);
		//pierrot.closePliers();
		//pierrot.turn180Degres(pierrot.LEFT);
		//pierrot.turn360Degres(pierrot.RIGHT);
		//pierrot.goal();
		//pierrot.turn90Degres(pierrot.RIGHT);
		//pierrot.goal();
		/*pierrot.getUltrasonics().getDistance().fetchSample(pierrot.getUltrasonics().getSample(), 0);
		if(pierrot.getUltrasonics().objectDetectedInRange(100))
			pierrot.catchTarget((int)(pierrot.getUltrasonics().getSample()[0] * 100));
		Delay.msDelay(1000);*/
		
		// attrapper un objet sans exception
		//pierrot.openPliers(); 
		
		//pierrot.test();
		//pierrot.fermerPinces();
		//pierrot.allera(1500,1200);
		//pierrot.allera(1000,0);
		//pierrot.rotate(360);
		pierrot.allera(500,600);
		//pierrot.rotate(180);+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		// pierrot.research();
		// pierrot.allerjusqua("BLANC");
		//pierrot.avancer(-100);
		//pierrot.getMotor().afficheLargeur();
		//pierrot.getMotor().afficheLongueur();
		//pierrot.getMotor().afficheLongueur();
		Delay.msDelay(8000);
	/*	
	 //test les couleurs 
		boolean again =true;
		while (again) {
		 System.out.println("\nPress enter to detect a color...");
			Button.ENTER.waitForPressAndRelease();
			//System.out.println(pierrot.colorint());
			System.out.println("la couleur est "+pierrot.color());
			Delay.msDelay(1000);
			if(Button.ESCAPE.isDown()) {
				again = false;
			}

		}
*/
		 
	
		}
	
	    private static void log(final String msg)
	    {
	        System.out.println("log>\t" + msg);
	        
	    }
}