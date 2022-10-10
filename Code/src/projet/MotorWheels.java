package projet;

import lejos.hardware.motor.Motor;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;

public class MotorWheels {
	private Wheel motor1 ;
	private Wheel motor2 ; 
	private Chassis chassis ; 
	 
	public MotorWheels() {
		Wheel motor1 = WheeledChassis.modelWheel(Motor.A, 81.6).offset(-70);
		Wheel motor2 = WheeledChassis.modelWheel(Motor.D, 81.6).offset(70);
		chassis = new WheeledChassis(new Wheel[]{ motor1, motor2 }, WheeledChassis.TYPE_DIFFERENTIAL);
	}
	
	public void move(double distance) {
		chassis.travel(distance);
	}
	public void rotate(double degre) {
		chassis.rotate(degre);
	}
	public void tournedroit() {
		chassis.getLinearDirection();
	}

	public Wheel getMoteur1() {
		return motor1;
	}

	public void setMoteur1(Wheel motor1) {
		this.motor1 = motor1;
	}

	public Wheel getMoteur2() {
		return motor2;
	}

	public void setMoteur2(Wheel motor2) {
		this.motor2 = motor2;
	}
	
}