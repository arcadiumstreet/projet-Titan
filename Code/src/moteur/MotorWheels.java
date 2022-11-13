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
		this.boussole=0;
		this.longueur =0; 
		this.largeur=1000;
		pilot.setLinearSpeed(chassis.getMaxLinearSpeed()-100);
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
		chassis = new WheeledChassis(new Wheel[]{ motor1, motor2 }, WheeledChassis.TYPE_DIFFERENTIAL);
		pilot = new MovePilot(chassis);
		this.boussole=0;
		this.longueur =0; 
		pilot.setLinearSpeed(chassis.getMaxLinearSpeed()-50);
		pilot.setAngularSpeed(200);
	if ( i == 1) {
		this.largeur = 500;
	}else if (i == 2){
		this.largeur = 1000;
	}else if( i == 3) {
		this.largeur = 1500;
	}
	else {this.largeur=1000;}
	}

	/**
	 * @param distance
	 */
	public void maj_longueur_largeur(double distance) {
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
	 * met à jour la boussole en lui appliquant une rotation de i degres
	 * @param i en degre
	 */
	public void majBoussole(double i) {
		
		if(this.boussole >= 0) {
			if(i <= 0) {
				this.boussole += i;
			}else if(i > 0 && this.boussole+i >= 180) {
				double a = 180 - this.boussole;
				double b = i - a;
				this.boussole = (180 - b)*-1;
			}else {
				this.boussole += i; 
			}
		}else {
			if(i > 0) {
				this.boussole +=i;
			}else if(i < 0 && this.boussole - i >= -180) {
				double a = 180 +this.boussole;
				double b = i - a;
				this.boussole = 180 - b;
			}else {
				this.boussole +=i;
			}
		}
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
	public void forward() {//ne met pas a jour la longueur mais ne pas changer 
		pilot.forward();
	}

	/**
	 * 
	 * @param distance
	 * @param immediateReturn
	 */
	public void forward(double distance,boolean immediateReturn) {
		double dis= distance*1.5;
		pilot.travel(dis,immediateReturn);
		maj_longueur_largeur(distance);
	}
	/**
	 * 
	 * @param distance
	 */
	public void forward(double distance) {
		//distance=1000,66cm(*1.5)
		double dis= distance*1.5;
		pilot.travel(dis);
		maj_longueur_largeur(distance);
	}
	
	/**
	 * recule de 66*1.5cm pour distance=1000
	 * @param distance
	 */
	public void backward(double distance) {
		pilot.travel(-distance*1.5,true);
		maj_longueur_largeur(-distance);
	}
	
	public void backward(double distance,boolean b) {
		pilot.travel(-distance*1.5,b);
		maj_longueur_largeur(-distance);
	}
	
	public void rotate() {
		pilot.rotate(360*1.29, true);
	}
	
	/**
	 * fait 1 tours sur soi par la gauche avec un retour immediat
	 */
	public void rotateneg() {
		pilot.rotate(-360*1.29, true);
	}
	
	public void rotate(double degre,boolean i){
		double d=1.29*degre ;
		pilot.setAngularSpeed(200);
		pilot.rotate(d,i);
		majBoussole(degre);
	}
	
	/**
	 * pivote de 1.29*degre à une vitesse de rotation de 200 et met à jour la boussole
	 * 
	 * @param degre
	 */
	public void rotate(double degre) {
		double d=1.29*degre ;
		pilot.setAngularSpeed(200);
		pilot.rotate(d);
		majBoussole(degre);}
	
	/**
	 * Recalibre la boussole à 0°
	 */
	public void boussole_a_0() {
		//double d=1.29*degre 
		pilot.setAngularSpeed(200);
		if (boussole <180) {pilot.rotate(-boussole*1.31);}
		else pilot.rotate((180-boussole)*1.26);
		this.setBoussole(0);
	}


	public void goTo(double largeurF, double longueurF) {
		if(largeurF==this.largeur && longueurF==this.longueur) {
			System.out.println("J'y suis deja");
			return;
		}
		boussole_a_0();
		if (longueurF >= this.longueur && largeurF <= this.largeur) {
			System.out.println("pass 1");
			
			double a;
			if(this.largeur-largeurF == 0) {
				a = 0;
			}else if (longueurF-this.longueur == 0) {
				a = 90;
			}else {
				a = Math.toDegrees(Math.atan((this.largeur-largeurF)/(longueurF-this.longueur)));
			}
			rotate(a);
			forward(Math.sqrt((Math.pow(Math.abs(this.longueur-longueurF), 2)) + (Math.pow(Math.abs(this.largeur-largeurF), 2)) ),false);
			this.largeur = largeurF;
			this.longueur = longueurF;
		}else if (longueurF >= this.longueur && largeurF >= this.largeur) {
			//essayer optimiser ca 
			System.out.println("pass 2");
			rotate(-90);
			double a;
			if(longueurF-this.longueur == 0) {
				a = 0;
			}else if( largeurF - this.largeur == 0){
				a = 90;
			}else {	
				a = Math.toDegrees(Math.atan((longueurF-this.longueur)/(largeurF-this.largeur)));
			}
			rotate(a);
			forward(Math.sqrt((Math.pow(Math.abs(this.longueur-longueurF), 2)) + (Math.pow(Math.abs(largeurF-this.largeur), 2)) ),false);
			this.largeur = largeurF;
			this.longueur = longueurF;
		}else if (longueurF <= this.longueur && largeurF <= this.largeur) {
			System.out.println("pass 3");
			rotate(90);
			double a;
			if(longueurF - this.longueur == 0) {
				a = 0;
			}else if(this.largeur-largeurF == 0){
				a = 90;
			}else {
				a = Math.toDegrees(Math.atan((this.longueur-longueurF)/(this.largeur-largeurF)));
			}
			rotate(a);
			forward(Math.sqrt((Math.pow(Math.abs(longueurF-this.longueur), 2)) + (Math.pow(Math.abs(this.largeur-largeurF), 2)) ),false);
			this.largeur = largeurF;
			this.longueur = longueurF;
		}else {
			System.out.println("pass 4");
			rotate(180);
			double a;
			if(largeurF - this.largeur == 0) {
				a = 0;
			}else if(this.longueur-longueurF == 0) {
				a = 90;
			}else {
				a = Math.toDegrees(Math.atan((largeurF-this.largeur)/(this.longueur-longueurF)));
			}			
			rotate(a);
			forward(Math.sqrt((Math.pow(Math.abs(longueurF-this.longueur), 2)) + (Math.pow(Math.abs(largeurF-this.largeur), 2)) ),false);
			this.largeur = largeurF;
			this.longueur = longueurF;
		}	
	}
	
	/**
	 * Affiche la valeur de la boussole
	 */
	public void afficheBoussole() {
		System.out.println("boussole"+this.boussole);}

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
	
	public double angle() {
		return pilot.getMovement().getAngleTurned();
	}
		
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
	
	public double getLongueur() {
		return longueur;
	}

	public void setLongueur(double longueur) {
		this.longueur = longueur;
	}

	public double getLargeur() {
		return largeur;
	}

	public void setLargeur(double largeur) {
		this.largeur = largeur;
	}
}