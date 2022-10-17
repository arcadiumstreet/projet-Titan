package moteur;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.robotics.RegulatedMotor;

public class Pinces {

	private RegulatedMotor pinces;
	
	public Pinces(Port pincesPort) {
		pinces = new EV3LargeRegulatedMotor(pincesPort);
	}

	public void fermer(boolean t){
		pinces.setSpeed(10000);
		pinces.rotate(500, t);;
	}

	public void ouvrir() {
		pinces.setSpeed(10000);
		this.pinces.rotate(4*360,true);
	}
	
	public void setSpeed(int sp) {
		pinces.setSpeed(sp);
	}
}
