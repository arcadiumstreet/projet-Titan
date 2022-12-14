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
	
	/**
	 * methode strategie1 qui est appelée au depart du round lorsque tous les palets sont la
	 * @param p qui est le robot utilisée pour lancer cette strategie
	 * @param d va prendre 1 ou -1 en fonction de l'endroit de depart 
	 * @Objectif marque les 2 premiers palets (1,7)ou (3,9)
	 */
	public static void strategie1(Robot p, int d){
		int palet1, palet2;
		if (d == -1) {
			palet1 = 3;
			palet2 = 9;
		} else {
			palet1 = 1;
			palet2 = 7;
		}
		//palet 1 ou 3
		p.ouvrirPinces(true);
		p.forward(600);
		p.fermerPinces(true);
		p.rotate(45*d);
		if (d==-1) {
			p.forward(400);
		}
		else {p.forward(300);}
		p.goal(false);
		p.majPaletpresent(palet1);
		// palet 7 ou 9
		p.rotate(-150*d);
		p.catchTarget(280);
		p.goal(false);
		p.majPaletpresent(palet2);
	}
	

	/**
	 * methode strategie1a qui est appelee apres la strategie 1 et sachant que l'adversaire est parti au milieu
	 * @param d va prendre 1 ou -1 en fonction de l'endroit de depart 
	 * @param p qui est le robot utilisée pour lancer cette strategie.
	 * @ les palet 1,7 ou 3,9 ont été récupéré 
	 * @Objectif : récupéré le palet 4 ou 6 puis le palet 5 s'il est encore présent et si le palet 5 n'est plus présent alors il va au palet 9 ou 7 
	 */
	public static void strategie1a(Robot p,int d){
		//Si l'adversaire part au milieu 
		int palet1, palet2bis;
		if (d == -1) {
			palet1 = 6;
			palet2bis = 7;
		} else {
			palet1 = 4;
			palet2bis = 9;
		}
		//palet 4 ou 6
		p.rotate(-173*d);
		p.catchTarget(850);
		p.goal(false);
		p.majPaletpresent(palet1);
		//palet 5 
		p.rotate(-170*d);
		p.forward(550);
		p.research();
		if (p.distance() <= 500) {
			p.catchTarget(p.distance());
			p.majPaletpresent(5);
		} else { 
			//palet 9 ou 7
			p.rotate(80);
			p.forward(500);
			p.research();
			p.catchTarget(p.distance());
			p.majPaletpresent(palet2bis);
		}
		p.getMotor().setLargeur(p.getMotor().getLargeur()*1.15);
		p.goal(true);		
	}	
	

	/**
	 * methode strategie1a qui est appelee apres la strategie 1 et sachant que l'adversaire est parti du coté opposé du robot 
	 * @param d va prendre 1 ou -1 en fonction de l'endroit de depart 
	 * @param p qui est le robot utilisée pour lancer cette strategie
	 * @ palet 1,7 ou 3,9 ont été récupéré 
	 * @Objectif: récupéré le palet 8 puis le palet 5 s'il est encore présent et si le palet 5 n'est plus présent alors il va au palet 4 ou 6 
	 */
	public static void strategie1b(Robot p, int d){
		int palet2;
		if (d == 1) {
			palet2 = 4;
		} else {
			palet2 = 6;
		}
		//palet 8
		if(d==-1) {
			p.rotate(110);
			p.ouvrirPinces(true);
			p.forward(700);
		}else {
			p.rotate(-120*d);
			p.ouvrirPinces(true);
			p.catchTarget(650);}
			p.goal(true);
			p.majPaletpresent(8);
		//palet 5 s'il est présent et palet 4 ou 6 si il est plus présent
			p.rotate(170*d);
			p.forward(500);
			if(d<0) {p.rotate(-90);}
			else {p.rotate(40);}
			p.research();
			if(p.distance()<=500){
			p.catchTarget(p.distance());
			p.goal(true);}
			p.getMotor().setLargeur(p.getMotor().getLargeur()*1.1);
			p.majPaletpresent(5);
	}
	
	
	/**
	 * methode strategie2 qui est appelee apres la pause du round lorsqu'au moins encore 2 palet est bien placé
	 * @param p qui est le robot utilisée pour lancer cette strategie
	 * @param d d va prendre 45 ou -45 en fonction de l'endroit de depart
	 * @param palet1 représente le premier palet que le robot va aller chercher,il représente celui qui est le plus pres de notre ligne de but 
	 * @param palet2 représente le deuxieme palet que le robot va aller chercher, il represente cest celui qui est le plus pres de la ligne de but adverse
	 * @objectif : mettre deux palet choisi comme la strat 1 
	 */
	public static void strategie2(Robot p,int d,int palet1,int palet2){	
			p.ouvrirPinces(true);
			p.alleraupalet(palet1);
			p.majPaletpresent(palet1);
			p.fermerPinces(false);
			p.boussole_a_0();
			p.rotate(45*d);
			p.forward(350);
			p.goal(false);
			p.ouvrirPinces(false);
			p.ouvrirPinces(true);
			p.alleraupalet(palet2);
			p.majPaletpresent(palet2);
			p.fermerPinces(false);
			p.fermerPinces(true);
			p.goal(true);
	}
	
	/**
	 * methode strategie3 qui est appelee apres les 2 premieres stratégies ou en reprise apres un temps mort quand aucun des palet est bien placé 
	 * @param p qui est le robot utilisée pour lancer cette strategie
	 * @param zone représente l'endroit ou le palet va lancer sa recherche (contient 6 zones de recherche)
	 * @objectif : trouver les palets restant en fonction de ce qui reste 
	 * 
	 * @plan_de_recherche:
	 * |-------|
	 * |  2 1  |
	 * |  3 4  |
	 * |  6 5  |
	 * |-------|
	 */
	public static void strategie3(Robot p,int zone ){
		int i=1;
		while(i!=3){
			
		for (;zone<7;zone++) {
			p.AllerAZone(zone);
			if(zone ==1||zone==2) {
				p.boussole_a_0();
				p.rotate(70);
			}
			p.research();
			if (p.distance()<=450) {
				p.catchTarget(p.distance());
				if (p.aunpalet()){
					p.goal(true);
				}
			}
		}
		zone =1;
		i++;
		}
		
	}
	
	public static void main(String[] args) {
		int placement=0;
		System.out.println("Ou est le robot ?");
		while(placement==0) {
			if(Button.RIGHT.isDown()) {
				placement=1;
			}
			if(Button.ENTER.isDown()) {
				placement=2;
			}
			if(Button.LEFT.isDown()) {
				placement=3;
			}
		}
		Delay.msDelay(500);
		System.out.println("On utilise la strat 3 ou 2 ou 1 ?");
		int strat=0;
		while(strat==0) {
			if(Button.RIGHT.isDown()) {
				strat=1;}
			if(Button.ENTER.isDown()) {
				strat=2;}
			if(Button.LEFT.isDown()) {
				strat=3;}
		}
		
		Delay.msDelay(100);
		Robot pierrot = new Robot(MotorPort.B,MotorPort.C,MotorPort.A,SensorPort.S4,SensorPort.S3,placement);
							
		System.out.println("Fermer(droite) ou ouvrir(gauche)les pinces ?");
		
		while(!Button.ESCAPE.isDown()) {
			if(Button.RIGHT.isDown()) {
				pierrot.getPinces().reglagepinces(-100);
			}
			if(Button.LEFT.isDown()) {
				pierrot.getPinces().reglagepinces(100);
			}
		}
		Delay.msDelay(200);
		
		
		int stratbis =0;	
		if(strat==1){
		System.out.println("l'adversaire part au milieu, utiliser 1A");
		while (stratbis==0){
			if(Button.ENTER.isDown()) {
				stratbis=1;
				System.out.println("OUI");
			}
			if(Button.ESCAPE.isDown()){
				stratbis=2;
				System.out.println("NON");
			}
		}
		Delay.msDelay(200);
		boolean c = true;
		if(stratbis==2){
		System.out.println("utiliser la strategie 1B");
		while(c) {
			if(Button.ENTER.isDown()) {
				stratbis=2;
				c=false;
			}
			if(Button.ESCAPE.isDown()) {
				stratbis=3;
				c=false;
			}
		}
		Delay.msDelay(200);
		}
		}

		int palet1=1,palet2=9;
		if(strat==2||strat==3) {
			int i=1;
			boolean t = false;
		while (i<=9) {
			System.out.println("le palet "+i+"  est present?");
			t=true;
			while (t) {
			if(Button.ENTER.isDown()){
				System.out.println("palet "+i+" OUI");
				t=false;}
			if(Button.ESCAPE.isDown()){
				pierrot.majPaletpresent(i);
				t=false;
				System.out.println("palet "+i+" NON");
				}
			}
			Delay.msDelay(500);
			i++;
		}
		pierrot.affichepaletpresent();}
		if(strat==2) {
		System.out.println("Premier palet a prendre?");
		boolean o =true;
		boolean b =true;
		while(o){
			if (b==true) {
				System.out.println("Palet "+palet1);
				b=false;
			}
			if(Button.ENTER.isDown()){
				o=false;
				System.out.println("oui");
			}
			if(Button.ESCAPE.isDown()){
				palet1++;
				b=true;
			}
			Delay.msDelay(200);
		}
		System.out.println("Dernier palet prit ?");
		o=true;
		b=true;
		while(o){
			if (b==true) {
				System.out.println("Palet"+palet2);
				b=false;
			}
			if(Button.ENTER.isDown()){
				o=false;
			}
			if(Button.ESCAPE.isDown()){
				palet2--;
				b=true;
			}
			Delay.msDelay(200);
		}
	}
	
	int zone =1;
	if(pierrot.paletpresent(7)||pierrot.paletpresent(8)) {
		zone=1;}
	else if(pierrot.paletpresent(9)){
		zone=2;}
	else if(pierrot.paletpresent(6)||pierrot.paletpresent(5)){
		zone=3;}
	else if(pierrot.paletpresent(4)){
		zone=4;	
	}else if(pierrot.paletpresent(2)||pierrot.paletpresent(1)){
		zone=5;
	}else {zone=6;}	
	/*	
		int zone =1;
		if (strat==3) {
			int o=1;
		boolean f = true;
		boolean fini = true;
			while (o<=6 && fini) {
				System.out.println("commencer sur la recherche zone "+o);
				f=true;
				while (f) {
				if(Button.ENTER.isDown()){
					System.out.println("OUI");
					zone=o;
					f=false;
					fini=false;
					}
				if(Button.ESCAPE.isDown()){
					f=false;
					System.out.println("NON");
					}
				}
				o++;
				Delay.msDelay(200);
			}
		}
		*/
		System.out.println();
		System.out.println("Pierrot pret a partir !");
		System.out.println("la strategie utilisee : "+strat);
		if(strat==1) {String res = "";
		if(stratbis==1) {res=" puis 1A ";}
		if(stratbis==2) {res=" puis 1B ";}
		if(stratbis==3) {res=" puis strat 3 ";}
			System.out.print(res);}
		if(strat==2) {System.out.println("   au palet "+palet1+" puis au palet "+palet2);}
		if(strat==3) {System.out.println("recherchArea "+zone);}
		Button.ENTER.waitForPressAndRelease();
		int d;
		if (placement == 3) {
			d = -1;
		} else {
			d = 1;
		}
		if(strat==1) {
			strategie1(pierrot,d);
			if(stratbis==1) {
				strategie1a(pierrot,d);
				zone=1;
				strategie3(pierrot,3);
			}
			else if(stratbis==2) {
				strategie1b(pierrot,d);
				zone=3;
				strategie3(pierrot,1);}
		}
		if(strat==2) {
			strategie2(pierrot,d,palet1,palet2);
			strategie3(pierrot,zone);
			}
		if(strat==3) {
			strategie3(pierrot,zone);
			}
		}
}