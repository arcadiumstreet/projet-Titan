package moteur;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.BrickFinder;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.motor.BaseRegulatedMotor;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.TachoMotorPort;
import lejos.robotics.Color;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.chassis.Wheel;
import lejos.utility.Delay;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.navigation.MovePilot.*;


public class MotorWheels {
	
	private Wheel motor1 ;
	private Wheel motor2 ; 
	private Chassis chassis ; 
	private MovePilot pilot ;

	private double boussole;
	private double longueur;
	private double largeur;
	 
	public MotorWheels(Port port,Port port2) {
		EV3LargeRegulatedMotor m1 = new EV3LargeRegulatedMotor(port);
		EV3LargeRegulatedMotor m2 = new EV3LargeRegulatedMotor(port2);
		
		Wheel motor1 = WheeledChassis.modelWheel(m1, 81.6).offset(-70);
		Wheel motor2 = WheeledChassis.modelWheel(m2, 81.6).offset(70);
		chassis = new WheeledChassis(new Wheel[]{ motor1, motor2 }, WheeledChassis.TYPE_DIFFERENTIAL);
		pilot = new MovePilot(chassis);
		boussole=0;
		this.longueur =1000; 
		this.largeur=0;
		//chassis.setAngularSpeed(chassis.getMaxLinearSpeed());
		//chassis.setVelocity(10000, 10000);	
		pilot.setLinearSpeed(chassis.getMaxLinearSpeed()-50);
		pilot.setAngularSpeed(200);
		}
	
<<<<<<< HEAD
	public MotorWheels(int i) { //i = 1 a gauche i=2 au milieu i=3 a droite
		Wheel motor1 = WheeledChassis.modelWheel(Motor.B, 81.6).offset(-70);
		Wheel motor2 = WheeledChassis.modelWheel(Motor.C, 81.6).offset(70);
=======
	public MotorWheels(Port port,Port port2,int i) { //i = 1 a gauche i=2 a milieu i=3 a droite
		EV3LargeRegulatedMotor m1 = new EV3LargeRegulatedMotor(port);
		EV3LargeRegulatedMotor m2 = new EV3LargeRegulatedMotor(port2);
		Wheel motor1 = WheeledChassis.modelWheel(m1, 81.6).offset(-70);
		Wheel motor2 = WheeledChassis.modelWheel(m2, 81.6).offset(70);
>>>>>>> 072a8d6854590242cb9fe7986d81e57da3b7a42c
		chassis = new WheeledChassis(new Wheel[]{ motor1, motor2 }, WheeledChassis.TYPE_DIFFERENTIAL);
		boussole=0;
		this.longueur =1000; 
	if ( i == 1) {
		this.largeur = 500;
	}else if (i == 2){
		this.largeur = 1000;
	}else if( i == 3) {
		this.largeur = 1500;
	}}

	public void mettre_a_jour_longueur_largeur(double distance) {
		System.out.println("Distance="+(int)distance);
		System.out.println("Boussole="+this.boussole);
		if(boussole > 0 && boussole < 90) {
			largeur -= Math.sin(Math.toRadians(boussole))*distance;
			longueur += Math.cos(Math.toRadians(boussole))*distance; 
		}else if (boussole < 0 && boussole > -90) {
			largeur += Math.sin(Math.abs(Math.toRadians(boussole)))*distance;
			longueur += Math.cos(Math.abs(Math.toRadians(boussole)))*distance; 
		}else if (boussole == 0) {
			largeur += 0;
			longueur += distance; 
		}else if (boussole == 90) {
			largeur -= distance;
			longueur += 0; 
		}else if (boussole == -90) {
			largeur += distance;
			longueur += 0;
		}else if (boussole > -180 && boussole < -90) {
			double angle = -boussole-90; 
			largeur += Math.cos(Math.toRadians(Math.abs(angle)))*distance;
			longueur -= Math.sin(Math.toRadians(Math.abs(angle)))*distance;
		}else if (boussole > 90 && boussole < 180) {
			double angle = boussole-90;
			largeur -= Math.cos(Math.toRadians(angle))*distance;
			longueur -= Math.sin(Math.toRadians(angle))*distance;
		}else if (boussole == 180 || boussole == -180) {
			largeur += 0;
			longueur -= distance; 
		}
		System.out.println("aL "+this.longueur);
		System.out.println("al "+this.largeur);
	}
	
	public void stop() {
		pilot.stop();
	}
	public void forward() {
		pilot.forward();
	}
	public void forward(double distance) {
		//distance=1000,66cm(*1.5)
		pilot.travel(distance*1.5);
	}
	
	public void backward(double distance) {
		pilot.travel(-distance*1.5);
	}
	
	public void boussole_a_0() {
		//double d=1.29*degre ;//a refaire sur le plan de jeu car pas la meme surface
		pilot.setAngularSpeed(200);//jamais le changer
		if (boussole <180) {pilot.rotate(-boussole*1.29);}
		else pilot.rotate((180-boussole)*1.29);
		this.setBoussole(0);
	}
	
	public void rotate(double angle,int i) {
		pilot.rotate(angle*1.29);
		if(angle==0)
			return;
		if(this.boussole<0 && angle>0){
			this.boussole+=angle;
			return;
		}
		if(this.boussole>0 && angle>0){
			if(this.boussole+angle > 180) {
				this.boussole = -360 +this.boussole+angle;
				return;
			}
			else {
				this.boussole += angle;
				return;
			}
		}
		if(this.boussole==0 && angle>0){
			this.boussole = angle;
			return;
		}
		if(this.boussole<0 && angle<0){
			if(this.boussole+angle<-180) {
				this.boussole = 360 - Math.abs(this.boussole+angle);
				return;
			}
			else {
				this.boussole += angle;
				return;
			}
		}
		if(this.boussole>0 && angle<0){
			this.boussole += angle;
			return;
		}
		if(this.boussole==0 && angle<0){
			this.boussole = angle;
			return;
		}		
	}
	
	public void rotateEnFonctionBoussole(double angleArrivee) { 

		double boussoleB;
		double angleB;
		if(angleArrivee == this.boussole) {
			return;
		}
		if(Math.abs(angleArrivee) == Math.abs(this.boussole) && Math.abs(angleArrivee) == 180) {
			return;
		}
		if(boussole < 0) {
			boussoleB = 360 + boussole;
		}else {
			boussoleB = boussole;
		}
		if(angleArrivee < 0) {
			angleB = 360 + angleArrivee;
		}else {
			angleB = angleArrivee;
		}
		if(angleB > boussoleB) {
			if(angleB - boussoleB > 180) { 
				rotate((angleB-360)-boussoleB);
			}else { //angleB - boussoleB <= 180
				rotate(angleB - boussoleB);
			}	
		}else {
			if(boussoleB - angleB < 180) {
				rotate(angleB - boussoleB);//
			}else { //boussoleB - angleB >= 180		
				rotate((360-boussoleB)+angleB);
			}
		}
		this.boussole = angleArrivee;
	}

	public void mettreAJourBoussole(double i) {
		
		if(this.boussole >= 0) {
			if(i <= 0) {
				//System.out.println("maj 1");
				this.boussole += i;
			}else if(i > 0 && this.boussole+i >= 180) {
				//System.out.println("maj 2");
				double a = 180 - this.boussole;
				double b = i - a;
				this.boussole = (180 - b)*-1;
			}else {
				this.boussole += i; 
			}
		}else {
			if(i > 0) {
				//System.out.println("maj 3");
				this.boussole +=i;
			}else if(i < 0 && this.boussole - i >= -180) {
				//System.out.println("maj 4");
				double a = 180 +this.boussole;
				double b = i - a;
				this.boussole = 180 - b;
			}else {
				this.boussole +=i;
			}
		}
	}
	
	public void afficheBoussole() {
		System.out.println(this.boussole);}

	public void afficheLongueur() {
		System.out.println("LONGUEUR " + this.longueur);}
	
	public void afficheLargeur() {
		System.out.println("LARGEUR "+ this.largeur);}
	
	public void move(double distance) {
		pilot.travel(distance);}
	
	public void rotate() {
		pilot.rotate(360*2, true);
	}
	
	public void rotate(double degre) {
		double d=1.29*degre ;//a refaire sur le plan de jeu car pas la meme surface
		pilot.setAngularSpeed(200);
		pilot.rotate(d);
		mettreAJourBoussole(degre);}
	
	public boolean enMouvement() {
		return chassis.isMoving();}
	
	public Wheel getMoteur1() {
		return motor1;}
	
	public Wheel getMoteur2() {
		return motor2;}
	
	public Chassis getChassis() {
		return chassis;}

	public void setMoteur1(Wheel motor1) {
		this.motor1 = motor1;}
	
	public void setMoteur2(Wheel motor2) {
		this.motor2 = motor2;}
	
	public void setChassis(Chassis c) {
		chassis=c;}

	public double getBoussole() {
		return boussole;}

	public void setBoussole(double boussole) {
		this.boussole = boussole;}
	
	public MovePilot getPilot() {
		return pilot;}

	public void setPilot(MovePilot pilot) {
		this.pilot = pilot;}
	
}