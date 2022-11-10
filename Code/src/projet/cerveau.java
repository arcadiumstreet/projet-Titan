package projet;

import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.motor.MindsensorsGlideWheelMRegulatedMotor;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.EncoderMotor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import java.math.*;
import lejos.hardware.sensor.EV3TouchSensor;


public class cerveau {
	//9 8 7//
	//6 5 4//
	//3 2 1//
	/**
	 * methode strategie1 qui est appelee au depart du round lorsqu tous les palets sont la
	 * @param d va prendre 1 ou -1 en fonction de l'endroit de depart 
	 * @param placement va prendre 1,2,3 en fonction de si il est a gauche au mileu ou a droite 
	 * @param angle angle vers lequel s'orienter pour trouver le 2 eme palet (140 ou -140)
	 */
	
		//ajouter la position de l'adversaire en pour savoir les palets pris par l'adversaire
	public static void strategie1(Robot p, int placement){
		//voir si on a le droit de partir les pinces ouvertes
		//marque les 2 premiers palets (1,7)ou (3,9)
		//1er premier palet 1
		int d, palet1, palet2;
		if (placement == 1) {
			d = -1;
			palet1 = 3;
			palet2 = 9;
		} else {
			d = 1;
			if (placement == 2) {
				palet1 = 2;
				palet2 = 8;
			} else {
				palet1 = 1;
				palet2 = 7;
			}
		}
		p.ouvrirPinces(true);
		p.forward(600);
		p.fermerPinces(true);
		p.rotate(45*d);
		p.forward(300);
		p.goal(false);
		p.majPaletpresent(palet1);
		// palet 7
		p.rotate(-140*d);
		p.catchTarget(350);
		p.goal(true);
		p.majPaletpresent(palet2);
	}
	
	/**
	 * methode strategie1 qui est appelee au depart du round lorsqu tous les palets sont la
	 * @param d va prendre 1 ou -1 en fonction de l'endroit de depart 
	 * @param placement va prendre 1,2,3 en fonction de si il est a gauche au mileu ou a droite 
	 * @param angle angle vers lequel s'orienter pour trouver le 2 eme palet (140 ou -140)
	 */
	
	public static void strategie1a(Robot p,int d, int placement){
		//si ladversaire ce met au milieu 
<<<<<<< HEAD
		//on on fait la strat 1 ou on prend 1,7 ou (3,9)
		//on va a 4 puis 5 puis 9 ou (6 puis a 5 puis a 7)
		
=======
		//on on fait la strat 1 ou on prend 1,7
		//on va a 4 puis a 5 ou 9 (en fonction de si le 5 est prit ou non)
>>>>>>> 1d52f67fed69cbba671af4f688f34d9f79591826
		//palet 4
		p.rotate(180*d);//a tester
		p.catchTarget(1000);//
		p.goal(true);
		p.majPaletpresent(4);
		//palet 5
		p.demitour();//
		p.forward(900);//
		p.research();
		p.catchTarget(p.distance());//
		p.goal(true);
		p.majPaletpresent(5);
		//palet 9
		p.demitour();	
	}	
	
	public static void strategie1b(Robot p,int d, int placement){
		//si ladversaire se met au coté opposé
		//on on fait la strat 1 ou on prend 1,7 ou 3,9
<<<<<<< HEAD
		// dnc on va chercher le 8 puis 5 ou
		//on va a 8
=======
		//on va a 8 puis a 5 ou 4 (en fonction de si le 5 est prit ou non)
>>>>>>> 1d52f67fed69cbba671af4f688f34d9f79591826
			//palet 8
				p.rotate(120*d);//a tester
				p.catchTarget(1000);//
				p.goal(true);
				p.majPaletpresent(8);
			//palet 5
				p.demitour();//
				p.forward(900);//
				p.research();
				p.catchTarget(p.distance());//
				p.goal(true);
				p.majPaletpresent(5);	
	}
	
	/**
	 * methode strategie2 qui est appelee apres la pause du round lorsqu'au moins encore un palet est bien plac�
	 * @param d d va prendre 45 ou -45 en fonction de l'endroit de depart 
	 * @param d2 va prendre 45 ou -45 en fonction de l'endroit de depart
	 * @param placement va prendre 0,1,2 en fonction de si il est a gauche au mileu ou a droite 
	 * @param angle angle vers lequel s'orienter pour trouver le 2 eme palet (155 ou -155)
	 */
	public static void strategie2(Robot p,int d, int placement){	
		//reprise apres la pause donc savoir quelle sont les palets restant mettre a jour paletpresent()
		//meme strat que 1 mettre le premier palet en dure 
		//savoir ceux qui reste 
		//aller sur le lequelle
		//prendre celui le plus pres du robot
		p.forward();
		while(!p.isPressed()) {
		}
		p.stop();
		//p.stop() ,changer pour aller plus vite
		p.fermerPinces(true);
		// pas obligatoire si aucun palet a esquivé  
		p.rotate(45*d);
		p.forward(200);
		p.goal(false);
		//a develloper
	}
	
	public static void strategie3(Robot p,int d, int placement){
		//quand aucun des palets est a sa place 
		//utiliser que research() pour aller au palets
		
		
	}
	
	
	public static void main(String[] args) {
		int placement,strat;
		placement=strat=0;
		System.out.println("Ou est le robot ?");
		while(placement==0) {
			if(Button.RIGHT.isDown()) {
				placement=3;
			}
			if(Button.LEFT.isDown()) {
				placement=1;
			}
			if(Button.ENTER.isDown()) {
				placement=2;
			}
		}
		
		System.out.println("Doit-on utiliser la stratégie 1 ?");
		while(strat==0) {
			if(Button.ENTER.isDown()) {
				strat=1;
			}
			if(Button.ESCAPE.isDown())
				strat=2;
		}
		
		Robot pierrot = new Robot(MotorPort.B,MotorPort.C,MotorPort.A,SensorPort.S4,SensorPort.S3,placement);
							
		boolean b=true;
		System.out.println("Fermer les pinces ?");
		while( b) {
			if(Button.ENTER.isDown()) {
				pierrot.fermerPinces(true);;
				b=!b;
			}
			if(Button.ESCAPE.isDown()) {
				b=!b;
			}
		}
		
		System.out.println("Pierrot pret à partir!");
		Button.ENTER.waitForPressAndRelease();
		
		if(placement==1) {
			if(strat==1)
				strategie1(pierrot,-1,placement);
			if(strat==2)
				strategie2(pierrot,-1,placement);
		}
		if(placement==3||placement==2) {
			if(strat==1)
				strategie1(pierrot,1,placement);
			if(strat==2)
				strategie2(pierrot,1,placement);
		}
	
		}
}