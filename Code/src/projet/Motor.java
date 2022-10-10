package projet;

import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;

public class Motor {
	private Wheel moteur1 ;
	private Wheel moteur2 ; 
	private Chassis chassis ; 
	 
	
	public Motor() {
		Wheel moteur1 = WheeledChassis.modelWheel(Motor.A, 81.6).offset(-70);
		Wheel moteur2 = WheeledChassis.modelWheel(Motor.D, 81.6).offset(70);
		chassis = new WheeledChassis(new Wheel[]{ moteur1, moteur2 }, WheeledChassis.TYPE_DIFFERENTIAL);
	}
	
	public void avance() {
		chassis.isMoving();
	}
	public void tournedroit() {
		
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
