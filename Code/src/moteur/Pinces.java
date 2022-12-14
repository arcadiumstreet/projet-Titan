package moteur;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.MindsensorsGlideWheelMRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.robotics.RegulatedMotor;


public class Pinces {

	private RegulatedMotor pinces;
	
	/**
	 * initiallise une instance de Pinces avec le port de son moteur et initiallise sa vitesse de fontionnement au maximum
	 * @param pincesPort le port sur lequel est branché le moteur des pinces
	 */
	public Pinces(Port pincesPort) {
		pinces = new MindsensorsGlideWheelMRegulatedMotor(pincesPort);
		pinces.setSpeed((int) pinces.getMaxSpeed());
	}
 
	/**
	 * regle les pinces de i degre 
	 * @param i un entier
	 */
	public void reglagepinces(int i) {
		pinces.rotate(i,false);
	}
	
	/**
	 * Ferme les pinces 3*330 degre
	 */
	public void fermer(boolean t){
		pinces.rotate(-3*330,t);
	}

	/** 
	 * Ouvre les pinces de 3*330 degre
	 */
	public void ouvrir(boolean t) {
		this.pinces.rotate(3*330,t);
	}
}