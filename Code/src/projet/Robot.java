package projet;

import java.util.Arrays;

import lejos.hardware.motor.MindsensorsGlideWheelMRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.robotics.Color;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;
import moteur.MotorWheels;
import moteur.Pinces;
import sensors.ColorSensor;
import sensors.UltrasonicSensor;
import sensors.TouchSensor;

public class Robot {

	private static Pinces pinces ;
	private static MotorWheels motor;
	
	private static UltrasonicSensor ultrasonics;
	private static TouchSensor touch; 
	private static ColorSensor color;
	private static final double[][] palet = {
			{500,600},{1000,600},{1500,600},
			{500,1200},{1000,1200},{1500,1200},
			{500,1800},{1000,1800},{1500,1800}};
	
	static double[][] researchArea = {
			{750,2100},{1250,2100},
			{1250,1500},{750,1500},
		 	{750,900}, {1250,900}};
	
	private static boolean[] paletpresent = {true,true,true,true,true,true,true,true,true};//pensez a l'initailiser au debut la partie 
	
	private static boolean aunpalet = false ;

	/**
	 * instancie un robot en initialisant : 
	 * ces roues, pinces, son detecteur ultrasonic, son detecteur de touché et son detecteur de couleur
	 * à partir des ports passés en paramètre ainsi que l'entier correspondant à la position sur le terrain (1:droit,2:milieu et 3:gauche)
	 * @param leftGearPort
	 * @param rightGearPort
	 * @param pliersPort
	 * @param ultrasonicsPort
	 * @param touchPort
	 * @param i
	 */
	public Robot(Port leftGearPort,Port rightGearPort,Port pliersPort,Port ultrasonicsPort,Port touchPort,int i){
		motor = new MotorWheels(leftGearPort,rightGearPort, i);
		pinces = new Pinces(pliersPort);
		ultrasonics = new UltrasonicSensor(ultrasonicsPort) ;
		touch = new TouchSensor(touchPort) ;
		color = new ColorSensor("S1");
	}
	/**
	 * 
	 * @return une chaine de charactère représentant un tableau 
	 * comprenant les valeur en entier des proportions de rouge, vert et bleu dans la couleur détectée par le capteur de couleur 
	 */
	public String colorint() {
        int [] e = color.getcolorint();
        String s = Arrays.toString(e) ;
         return s ;
	}
	
	/**
	 * @return une chaine de charactère représentant la couleur détectée par 
	 * le capteur de couleur ("NOIR", "GRIS", "JAUNE", "BLANC", "ROUGE", "VERT", "BLEU" OU "NON RECONNU")
	 */
	public String color() {
        Color rgb = ColorSensor.getColor();
        return ColorSensor.color_String(rgb.getRed(), rgb.getGreen(), rgb.getBlue());
    }
	
	/**
	 * fait avance le robot jusqu'à ce que le capteur detecte la couleur passée en paramètre
	 * 
	 * @param couleur en chaine de caractère à détecter
	 */
	public void allerjusqua(String couleur) {
		motor.forward();
	    Color rgb = ColorSensor.getColor();
	    while( ColorSensor.color_String(rgb.getRed(), rgb.getGreen(), rgb.getBlue()) != couleur){
	    	rgb = ColorSensor.getColor();
				if(!estunpalet()) {
				motor.stop();
				motor.rotate(45,false);
				motor.forward(300,false);
				motor.boussole_a_0();
				motor.forward();
				}
	    }		
	   motor.stop();
	} 
	
	/**
	 * 
	 * @param i
	 * @return
	 */
	public static double[] getzone(int i){
		return researchArea[i-1];}
	/**
	 * 
	 * @param researchArea
	 * @return
	 */
	public static double getzonelongueur(double[] researchArea){
		return researchArea[0];}
	/**
	 * 
	 * @param researchArea
	 * @return
	 */
	public static double getzonelargeur(double[] researchArea){
		return researchArea[1];}
	
	/**
	 * 
	 * @param zone
	 */
	public static void AllerAZone(int zone ) {
		motor.goTo(getzonelongueur(getzone(zone)),getzonelargeur(getzone(zone)));
	}
	/**
	 * 
	 * @param i le numéro d'un palet
	 * @return un tableau de double contenant les coordonnées du palet voulu
	 */
	public static double[] getpalet(int i){
		return palet[i-1];
	}
	
	/**
	 * 
	 * @param palet est un tableau de double contenant les coordonnées d'un palet 
	 * @return la longueur de l'emplacement du palet
	 */
	public static double getpaletlongueur(double[] palet){
		return palet[0];
		}
	
	/**
	 * 
	 * @param palet est un tableau de double contenant les coordonnées d'un palet 
	 * @return la largeur de l'emplacement du palet
	 */
	public static double getpaletlargeur(double[] palet){
		return palet[1];
	}
	
	/**
	 * passe le boolean du tableau paletpresent correspondant au palet dont le numéro est passé en paramètre sur false
	 * 
	 * @param i le numéro du palet à mettre à jour
	 */
	public static void majPaletpresent(int i) {
		paletpresent[i-1]=false;
	}

	/**
	 *  le robot va chercher le palet dont le numéro est passé en paramètre 
	 *  et indique dans le tableau paletpresent qu'il est deja récupéré (passe de true à false)
	 * @param i est le numéro du palet à récupérer
	 */
	public static void alleraupalet(int i) {
		if(paletpresent[i]) {
		motor.goTo(getpaletlongueur(getpalet(i)), getpaletlargeur(getpalet(i)));
		majPaletpresent(i);}
		else{
		motor.goTo(getpaletlongueur(getpalet(i+1)), getpaletlargeur(getpalet(i+1)));
		majPaletpresent(i+1);}
	}
	/**
	 * 
	 * @param i entier représentant le numéro d'un palet
	 * @return true si le palet est potentielement présent false sinon
	 */
	public static boolean paletpresent(int i) {
		return paletpresent[i-1];
	}
	
	/**
	 * avance jusqu'à l'appel de la methode stop()
	 */
	public void forward() {
		motor.forward();
	}

	/**
	 * le robot avance de d mm
	 * @param d un distance entière en mm
	 */
	public void forward(double d) {
		motor.forward(d);
	}
	
	/**
	 * le robot recule de d
	 * @param d une distance entière en mm
	 */
	public void backward(double d) {
		motor.backward(d);
	}

	/**
	 * le robot recule de d mm tout en continiant d'exécuter la suite du code si b vaut true
	 * sinon il recule juste de d mm
	 * @param d distance entière en mm
	 * @param b boolean indiquant si le mouvement est synchrone ou non
	 */
	public void backward(double d,boolean c) {
		motor.backward(d,c);
	}
	
	/**
	 * le robot avance de d mm tout en continiant d'exécuter la suite du code si b vaut true
	 * sinon il avance juste de d mm
	 * @param d d distance entière en mm
	 * @param b boolean valant true pour un retour instantané false sinon
	 */
	public void forward(double d,boolean b) {
		motor.forward(d,b);
	}
	
	/**
	 * le robot exécute un demitour(rotation de 180° vers la droite)
	 */
	public void demitour() {
		motor.rotate(180);
	}
	
	/**
	 * le robot tourne de d degre
	 * @param d degre de rotation, positif vers la droite et négatif vers la gauche
	 */
	public void rotate(double d) {
		motor.rotate(d);
	}
	
	/**
	 * le robot tourne de d degre tout en exécutant la suite du code si b vaut true
	 * sinon effectue seulement la rotation de d degre
	 * @param d degre de rotation, positif vers la droite et négatif vers la gauche
	 * @param b boolean indiquant si le mouvement est syncrone ou non
	 */
	public void rotate(double d,boolean b) {
		motor.rotate(d,b);
	}
	
	/**
	 * le robot tourne de d degre tout en exécutant la suite du code si b vaut true
	 * sinon effectue seulement la rotation de d degre
	 * @param d degre de rotation, positif vers la gauche et négatif vers la droite
	 * @param b boolean indiquant si le mouvement est syncrone ou non
	 */
	public void rotateneg(double d,boolean b) {
		motor.rotate(-d,b);
	}
	
	/**
	 * le robot fait un tour complet sur lui meme (rotation de 360°)
	 */
	public void lap() {
		motor.rotate();
	}
	
	/**
	 * recalibre la boussole à 0° et oriente le robot dans cette direction
	 */
	public void boussole_a_0() {
		motor.boussole_a_0();
	}
	
	/**
	 * le robot se rend au coordonnées de largeur x et longueur y
	 * @param x un double représentant la largeur de la coordonnée de destination
	 * @param y un double représentant la longueur de la coordonnée de destination
	 */
	public void allerA(double x, double y) {
		motor.goTo(x, y);  
	}

	/**
	 * ouvre les pinces, attrape le palet dont la distance au robot a été passée en paramètre puis ferme les pinces
	 * @param targetDistance est la distance en mm au palet à attraper
	 */
	public void catchTarget(float targetDistance){
		ouvrirPinces(true);
		motor.forward(targetDistance + 40,true);
		while(estunpalet() && motor.enMouvement() ){}
		motor.stop();
		if(!estunpalet()){
			aunpalet=false;
			System.out.println("es la"+distance());
			backward((targetDistance + 40)-150,false);
			motor.maj_longueur_largeur(-350);
		}else aunpalet=true;
		fermerPinces(false);
	}
	
	/**
	 * tout objet détecté à plus de 150 mm est considéré comme un palet et ceux en dessous non
	 * @return true si l'objet détecté est à plus de 150 mm et false si non
	 */
	public boolean estunpalet() {
		float dis;
		getUltrasonics().getDistance().fetchSample(getUltrasonics().getSample(), 0);
		dis = getUltrasonics().getSample0();
		return dis > 0.15 ;
	}
	
	/**
	 * le robot tourne sur lui meme jusqu'à détecter un object à moins de 500mm  
	 */
	public void research(){
		boolean tourcomplet = false;
		float dis=1;
		motor.getPilot().setAngularSpeed(150);
		motor.rotate();
		long t1= System.currentTimeMillis();
		while(dis>0.45 && motor.enMouvement()){
			getUltrasonics().getDistance().fetchSample(getUltrasonics().getSample(), 0);
		dis = getUltrasonics().getSample0();}
		long t2 = System.currentTimeMillis();
		if(!motor.enMouvement()) {
			tourcomplet=true;
		}
		motor.stop();
		long temps = t2-t1 ;
		long coeff = 0;
		if(temps<=900) {coeff= (long) 11;
		}
		if(temps>900&&temps<=1800) {coeff= (long) 10.01;
		}
		if(temps>1800&&temps<=2700) {coeff= (long) 9.47;
		}
		if(temps>2700) {coeff=(long)9.28;}
		long angle = (temps/coeff)+15;
		System.out.println("angle = "+angle);
		System.out.println("distance "+dis*1000);
		motor.getPilot().setAngularSpeed(200);
		if(tourcomplet) {}
		else {motor.majBoussole(angle);}	
	}

	/**
	 * 
	 * @return la distance(en mm) à laquelle le robot détecte quelque chose, return Float.POSITIVE_INFINITY si >2.55m 
	 */
	public float distance() {
		float dis=1;
		getUltrasonics().getDistance().fetchSample(getUltrasonics().getSample(), 0);
		dis = getUltrasonics().getSample0();
		if (dis==Float.POSITIVE_INFINITY) {dis=100;}
		return dis*1000 ;
	}

	/**
	 * le robot va marquer le palet dans ses pinces dans le but adverse et recalibre la boussole si le boolean en paramétre vaut true sinon ne recalubre pas 
	 * @param b boolean valant true s'il faut effectuer un recalibrage de la boussole et false sinon
	 */
	public void goal(boolean b) {
		motor.boussole_a_0();
		allerjusqua("BLANC");
		if(b){erreurs_boussole();}
		ouvrirPinces(false);
		motor.setLongueur(2230);
		motor.backward(200,true);
		fermerPinces(false);
		motor.afficheLongueur();
		aunpalet=false;
	}
	/**
	 * 
	 * @return true su l'objet dont il s'approche est effectivement un palet
	 */
	public static boolean aunpalet() {
		return aunpalet;
	}


	
	/**
	 * recalibre la boussole afin de limiter les erreurs
	 */
	public void erreurs_boussole() {
		
        motor.rotate(30,false);
        motor.rotate(-60,true);
        double min = 100;
        double angle = 0;
        while(motor.enMouvement()) {
        	getUltrasonics().getDistance().fetchSample(getUltrasonics().getSample(), 0);
            double valeur= getUltrasonics().getSample0();
            if(valeur<min) {
                min=valeur;
                angle= motor.angle();
            }
            Delay.msDelay(3);
        }
        System.out.println("angle trouver "+angle);
        System.out.println("valeurs min"+min);
        motor.rotate(60+angle, false);
        motor.setBoussole(0);
    }

	/**
	 * 
	 * @return pinces
	 */
	public Pinces getPinces() {
		return pinces ;
	}

	/**
	 * le robot ouvre les pinces et continue à executer la suite du programme simultanément si le paramétre vaut true,
	 * sinon fini d'ouvrire ses pinces avec d'exécuter la suite du programme
	 * @param t boolean indiquant si l'action est synchrone ou non
	 */
	public void ouvrirPinces(boolean t) {
		pinces.ouvrir(t);
	}

	/**
	 * le robot ferme les pinces et continue à executer la suite du programme simultanément si le paramétre vaut true,
	 * sinon fini de fermer ses pinces avec d'exécuter la suite du programme
	 * @param t boolean indiquant si l'action est synchrone ou non
	 */
	public void fermerPinces(boolean t) {
		pinces.fermer(t);
	}

	/**
	 * 
	 * @return motor
	 */
	public static MotorWheels getMotor() {
		return motor;
	}

	/**
	 * affecte à l'attribut motor l'instance de MotorWheels passé en paramètre
	 * @param motor une instance de MotorWheels
	 */
	public static void setMotor(MotorWheels motor) {
		Robot.motor = motor;
	}
	/**
	 * 
	 * @return ultrasonucs
	 */
	public static UltrasonicSensor getUltrasonics() {
		return ultrasonics;
	}

	/**
	 * affecte à l'attribut ultrasonics l'instance de UltrasonicSensor passé en paramètre
	 * @param ultrasonics une instance d'UltrasonicSensor
	 */
	public void setUltrasonics(UltrasonicSensor ultrasonics) {
		this.ultrasonics = ultrasonics;
	}

	/**
	 * 
	 * @return touch
	 */
	public EV3TouchSensor getTouch() {
		return touch;
	}

	/**
	 * affecte à l'attribut touch l'instance de TouchSensor passé en paramètre
	 * @param touch une instance de TouchSensor
	 */
	public void setTouch(TouchSensor touch) {
		this.touch = touch;
	}

	/**
	 * 
	 * @return color
	 */
	public ColorSensor getColor() {
		return color;
	}

	/**
	 * affecte à l'attribut color l'instance de ColorSensor passé en paramètre
	 * @param color une instance de ColorSensor
	 */
	public void setColor(ColorSensor color) {
		this.color = color;
	}
	
	/**
	 * stop les déplacements du robot
	 */
	public void stop() {
		motor.stop();
	}

	/**
	 * 
	 * @return true si ne détecteur de touche capte quelque chose
	 */
	public boolean isPressed() {
		return touch.isPressed();
	}

	/**
	 *affecte à l'attribut palerpresent un tableau de boolean passé en paramètre représentant 
	 *les palet abscent par des false et ceux potenciellementprésent par des true
	 * @param paletpresent un tableau de boolean
	 */
	public static void setPaletpresent(boolean[] paletpresent) {
		Robot.paletpresent = paletpresent;
	}

	
	/**
	 * 
	 * @return true si le robot est en mouvement false sinon
	 */
	public boolean enMouvement() {
		return motor.enMouvement();
	}
	/**
	 * retourn le tableau représentant la présence des palet sous la forme d'une chaind de caractère 
	 * @return une chaine de caractère
	 */
	public String affichepaletpresent() {
		return Arrays.toString(paletpresent);
	}
}