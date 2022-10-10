package projet;

import lejos.hardware.motor.Motor;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.hardware.port.Port;


public class MotorWheels {
	private Wheel moteur1 ;
	private Wheel moteur2 ; 
	private Chassis chassis ; 
	 
	public MotorWheels(Port A,Port D) {
		Wheel moteur1 = WheeledChassis.modelWheel(Motor.A, 81.6).offset(-70);
		Wheel moteur2 = WheeledChassis.modelWheel(Motor.D, 81.6).offset(70);
		chassis = new WheeledChassis(new Wheel[]{ moteur1, moteur2 }, WheeledChassis.TYPE_DIFFERENTIAL);
	}
	
	public boolean enMouvement() {
		return chassis.isMoving();
	}
	
	
	
	public void avance() {
		
	}
	
	public void tournedroit() {
		chassis.getLinearDirection();
	}

	public Wheel getMoteur1() {
		return moteur1;
	}

	public void setMoteur1(Wheel moteur1) {
		this.moteur1 = moteur1;
	}

	public Wheel getMoteur2() {
		return moteur2;
	}

	public void setMoteur2(Wheel moteur2) {
		this.moteur2 = moteur2;
	}
	
}