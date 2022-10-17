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
	 
	/**
	 * Initiallise une instance de MotorWheels avec
	 * motor1 la roue du chassis branchée sur port,
	 * motor2 la roue du chassis branchée sur port2,
	 * chassis le chassis comprennant les roue moteur1 et moteur2,
	 * pilot le pilot du chassis,
	 * boussole=0, longueur=1000,largeur= 0,
	 * la vitesse de déplacement du pilot à la vitesse maximum du chassis-50,
	 * la vitesse de rotation du pilot à 200
	 * 
	 * @param port le port auquel est branché le moteur droit
	 * @param port2 le port auquel est branché le moteur gauche
	 */
	public MotorWheels(Port port,Port port2) {
		EV3LargeRegulatedMotor m1 = new EV3LargeRegulatedMotor(port);
		EV3LargeRegulatedMotor m2 = new EV3LargeRegulatedMotor(port2);
		
		Wheel motor1 = WheeledChassis.modelWheel(m1, 81.6).offset(-70);
		Wheel motor2 = WheeledChassis.modelWheel(m2, 81.6).offset(70);
		chassis = new WheeledChassis(new Wheel[]{ motor1, motor2 }, WheeledChassis.TYPE_DIFFERENTIAL);
		pilot = new MovePilot(chassis);
		boussole=0;
		this.longueur =0; 
		this.largeur=1000;
		//chassis.setAngularSpeed(chassis.getMaxLinearSpeed());
		//chassis.setVelocity(10000, 10000);	
		pilot.setLinearSpeed(chassis.getMaxLinearSpeed()-50);
		pilot.setAngularSpeed(200);
		}
	

	/**
	 * Initiallise une instance de MotorWheels avec
	 * motor1 la roue du chassis branchée sur port,
	 * motor2 la roue du chassis branchée sur port2,
	 * chassis le chassis comprennant les roue moteur1 et moteur2,
	 * pilot le pilot du chassis,
	 * boussole=0, longueur=1000,
	 * largeur=500 si i vaut 1,
	 * largeur=1000 si i vaut 2,
	 * largeur=1500 si i vaut 3,
	 * la vitesse de déplacement du pilot à la vitesse maximum du chassis-50,
	 * la vitesse de rotation du pilot à 200
	 * @param port le port auquel est branché le moteur droit
	 * @param port2 le port auquel est branché le moteur gauche
	 * @param i entier relatif au placement du robot : i = 1 à gauche, i=2 au milieu, i=3 à droite;
	 * 
	 */
	public MotorWheels(Port port,Port port2,int i) { 
		EV3LargeRegulatedMotor m1 = new EV3LargeRegulatedMotor(port);
		EV3LargeRegulatedMotor m2 = new EV3LargeRegulatedMotor(port2);
		Wheel motor1 = WheeledChassis.modelWheel(m1, 81.6).offset(-70);
		Wheel motor2 = WheeledChassis.modelWheel(m2, 81.6).offset(70);
		pilot = new MovePilot(chassis);
		chassis = new WheeledChassis(new Wheel[]{ motor1, motor2 }, WheeledChassis.TYPE_DIFFERENTIAL);
		boussole=0;
		this.longueur =1000; 
	if ( i == 1) {
		this.largeur = 500;
	}else if (i == 2){
		this.largeur = 1000;
	}else if( i == 3) {
		this.largeur = 1500;
	}
	pilot.setLinearSpeed(chassis.getMaxLinearSpeed()-50);
	pilot.setAngularSpeed(200);
	}

	/**
	 * 
	 * @param distance
	 */
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
	
	/**
	 * stop les 2 roues du pilot
	 */
	public void stop() {
		pilot.stop();
	}
	
	/**
	 * avance jusqu'à lappel de la methode stop()
	 */
	public void forward() {
		pilot.forward();
	}

	/**
	 * 
	 * 
	 * @param distance
	 * @param immediateReturn
	 */
	public void forward(double distance,boolean immediateReturn) {
		pilot.travel(distance,immediateReturn);
		
	}

	/**
	 * 
	 * @param distance
	 */
	public void forward(double distance) {
		//distance=1000,66cm(*1.5)
		double dis= distance*1.5;
		pilot.travel(dis);
		mettre_a_jour_longueur_largeur(distance);
	}
	
	/**
	 * recule de 66*1.5cm pour distance=1000
	 * @param distance
	 */
	public void backward(double distance) {
		pilot.travel(-distance*1.5);
	}
	
	/**
	 * Recalibre la boussole à 0°
	 */
	public void boussole_a_0() {
		//double d=1.29*degre ;//a refaire sur le plan de jeu car pas la meme surface
		pilot.setAngularSpeed(200);//jamais le changer
		if (boussole <180) {pilot.rotate(-boussole*1.29);}
		else pilot.rotate((180-boussole)*1.29);
		this.setBoussole(0);
	}
	
	/**
	 * pivote de |angle|°, vers la droite si angle>0, vers la gauche si angle<0
	 * met à jour la boussole selon l'angle de rotation
	 * 
	 * @param angle en degre
	 * @param i
	 */
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
	
	/**
	 * pivot de façon à ce que la boussole soit égale à l'angle d'arrivé
	 * 
	 * @param angleArrivee en degre
	 */
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

	/**
	 * met à jour la boussole en lui appliquant une rotation de i degres
	 * 
	 * @param i en degre
	 */
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
	
	/**
	 * Affiche la valeur de la boussole
	 */
	public void afficheBoussole() {
		System.out.println(this.boussole);}

	/**
	 * affiche la valeur de la longueur
	 */
	public void afficheLongueur() {
		System.out.println("LONGUEUR " + this.longueur);}
	
	/**
	 * affiche la valeur de la largeur
	 */
	public void afficheLargeur() {
		System.out.println("LARGEUR "+ this.largeur);}
	
	
	/**
	 * fait 1 tours sur soi par la droite avec un retour immediat
	 */
	public void rotate() {
		pilot.rotate(360*1.29, true);
	}
	
	/**
	 * fait 1 tours sur soi par la gauche avec un retour immediat
	 */
	public void rotateneg() {
		pilot.rotate(-360, true);
	}
	
	/**
	 * pivote de 1.29*degre à une vitesse de rotation de 200 et met à jour la boussole
	 * 
	 * @param degre
	 */
	public void rotate(double degre) {
		double d=1.29*degre ;//a refaire sur le plan de jeu car pas la meme surface
		pilot.setAngularSpeed(200);
		pilot.rotate(d);
		mettreAJourBoussole(degre);}
	
	/**
	 * détect si le chassis est en mouvement
	 * @return true si le chassis est en mouvement, false sinon
	 */
	public boolean enMouvement() {
		return chassis.isMoving();}
	
	/**
	 * 
	 * @return motor1
	 */
	public Wheel getMoteur1() {
		return motor1;}
	
	/**
	 * 
	 * @return motor2
	 */
	public Wheel getMoteur2() {
		return motor2;}
	
	/**
	 * 
	 * @return chassis
	 */
	public Chassis getChassis() {
		return chassis;}

	/**
	 * attribut motor1 à l'attribut this.motor1
	 * 
	 * @param motor1
	 */
	public void setMoteur1(Wheel motor1) {
		this.motor1 = motor1;}
	
	/**
	 * attribut motor2 à l'attribut this.motor2
	 * 
	 * @param motor2
	 */
	public void setMoteur2(Wheel motor2) {
		this.motor2 = motor2;}
	
	/**
	 * affect chassis à l'attribut this.chassis
	 * 
	 * @param c
	 */
	public void setChassis(Chassis c) {
		chassis=c;}

	/**
	 * 
	 * @return la valeur de la boussole
	 */
	public double getBoussole() {
		return boussole;}

	/**
	 * affect la valaur de boussole à l'attribut this.boussole
	 * 
	 * @param boussole
	 */
	public void setBoussole(double boussole) {
		this.boussole = boussole;}
	
	/**
	 * 
	 * @return pilot
	 */
	public MovePilot getPilot() {
		return pilot;}

	/**
	 * affect pilot à l'attribut this.pilot
	 * 
	 * @param pilot
	 */
	public void setPilot(MovePilot pilot) {
		this.pilot = pilot;}
	
}