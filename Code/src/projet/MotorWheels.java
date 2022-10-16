package projet;

import lejos.hardware.motor.Motor;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.hardware.port.Port;


public class MotorWheels {
	private Wheel motor1 ;
	private Wheel motor2 ; 
	private Chassis chassis ; 
	 

	/*public MotorWheels(Port A,Port D) {
		Wheel moteur1 = WheeledChassis.modelWheel(Motor.A, 81.6).offset(-70);
		Wheel moteur2 = WheeledChassis.modelWheel(Motor.D, 81.6).offset(70);
		chassis = new WheeledChassis(new Wheel[]{ moteur1, moteur2 }, WheeledChassis.TYPE_DIFFERENTIAL);*/
	public MotorWheels() {
		Wheel motor1 = WheeledChassis.modelWheel(Motor.A, 81.6).offset(-70);
		Wheel motor2 = WheeledChassis.modelWheel(Motor.D, 81.6).offset(70);
		chassis = new WheeledChassis(new Wheel[]{ motor1, motor2 }, WheeledChassis.TYPE_DIFFERENTIAL);
	}
	
	public boolean enMouvement() {
		return chassis.isMoving();
	}
	
	
	
	public void avance(double dist) {
		chassis.travel(dist);
	}
	
	public void tournedroit() {
		chassis.getLinearDirection();
	}

	public Wheel getMoteur1() {
		return motor1;
	}
	
	public Wheel getMoteur2() {
		return motor2;
	}
	
	public Chassis getChassis() {
		return chassis;
	}

	public void setMoteur1(Wheel motor1) {
		this.motor1 = motor1;
	}

	
	public void setMoteur2(Wheel motor2) {
		this.motor2 = motor2;
	}
	
	public void setChassis(Chassis c) {
		chassis=c;
	}
	
}