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
	
	public Pinces(Port pincesPort) {
		pinces = new MindsensorsGlideWheelMRegulatedMotor(pincesPort);
		pinces.setSpeed((int) pinces.getMaxSpeed());
	}

	public void reglagepinces(int i) {
		pinces.rotate(i);
	}
	public void fermer(){
		pinces.rotate(-3*360);
	}

	public void ouvrir() {
		this.pinces.rotate(3*360);
	}

}

