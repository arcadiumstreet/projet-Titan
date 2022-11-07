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
	
	public static final int RIGHT = 1;
	public static final int FRONT = 1;
	public static final int LEFT = - 1;
	public static final int BACK = - 1;
	private static Pinces pinces ;
	private static MotorWheels motor;
	
	private static UltrasonicSensor ultrasonics;
	private static TouchSensor touch; 
	private static ColorSensor color;
	private static final double[][] palet = {
			{500,600},{1000,600},{1500,600},
			{500,1200},{1000,1200},{1500,1200},
			{500,1800},{1000,1800},{1500,1800}};
	
	private static boolean[] paletpresent = {true,true,true,true,true,true,true,true,};//pensez a l'initailiser au debut la partie 
	
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
	 * 
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
	 * @param couleur
	 */
	public void allerjusqua(String couleur) {
		motor.forward();
	    Color rgb = ColorSensor.getColor();
	    while( ColorSensor.color_String(rgb.getRed(), rgb.getGreen(), rgb.getBlue()) != couleur) {
	    	rgb = ColorSensor.getColor();
	    }
	   motor.stop();
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
	 *  le obot va chercher le palet dont le numéro est passé en paramètre 
	 *  et indique dans le tableau paletpresent qu'il est deja récupéré (passe de true à false)
	 * @param i est le numéro du palet à récupérer
	 */
	public static void alleraupalet(int i) {///mettre a jour 
		if(paletpresent[i-1]) {
		motor.goTo(getpaletlongueur(getpalet(i)), getpaletlargeur(getpalet(i)));
		majPaletpresent(i);}
		
	}
	
	
	/**
	 * avance jusqu'à lappel de la methode stop()
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
	 * le robot avance de d mm tout en continiant d'exécuter la suite du code si b vaut true
	 * sinon il avance juste de d mm
	 * @param d distance entière en mm
	 * @param b boolean indiquant si le mouvement est syncrone ou non
	 */
	public void forward(double d,boolean b) {
		motor.forward(d,b);
	}
	
	/**
	 * le robot exécute un demitour(rotation de 180° vers la droite
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
	public void allera(double x, double y) {
		motor.goTo(x, y);
	    //motor.moveTo(x,y);
	}

	/**
	 * ouvre les pinces, attrape le palet dont la distance au robot a été passée en paramètre puis ferme les pinces
	 * @param targetDistance est la distance en mm au palet à attraper
	 */
	public void catchTarget(float targetDistance){//// a voir 
		ouvrirPinces(true);
		motor.forward(targetDistance + 50,true);
		while((estunpalet() && motor.enMouvement() && !isPressed())){}
		fermerPinces(true);
	}
	
	/**
	 * tout objet détecté à plus de 150mm est considéré comme un palet et ceux en dessous non
	 * @return true si l'objet détécté est à plus de 150 mm et false si non
	 */
	public boolean estunpalet() {
		float dis;
		getUltrasonics().getDistance().fetchSample(getUltrasonics().getSample(), 0);
		dis = getUltrasonics().getSample0();
		return dis > 0.15 ;
	}
	
	/**
	 * le robot tourne sur lui meme jsuqu'à détécter un object à moins de 500mm  
	 */
	public void research(){
		float dis=1;
		motor.getPilot().setAngularSpeed(150);
		motor.rotate();
		long t1= System.currentTimeMillis();
		while(dis>0.5){
			getUltrasonics().getDistance().fetchSample(getUltrasonics().getSample(), 0);
			dis = getUltrasonics().getSample0();
			}
		long t2 = System.currentTimeMillis();
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
		motor.getPilot().setAngularSpeed(200);
		motor.majBoussole(angle);
	}

	public float distance() {
		float dis=1;
		getUltrasonics().getDistance().fetchSample(getUltrasonics().getSample(), 0);
		dis = getUltrasonics().getSample0();
		return dis*1000 ;
	}
	
	public void goal() {
		motor.boussole_a_0();
		allerjusqua("BLANC");
		erreurs_boussole();
		ouvrirPinces(false);
		//mettre en true avec du delay
		motor.backward(200);
		fermerPinces(true);
		motor.setLongueur(2300);//distance max-10cm(distance entre le capteur couleur et le centre du robot 
		motor.afficheLongueur();
	}
	
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

	public Pinces getPinces() {
		return pinces ;
	}
	
	public void ouvrirPinces(boolean t) {
		pinces.ouvrir(t);
	}
	public void fermerPinces(boolean t) {
		pinces.fermer(t);
	}
	public static MotorWheels getMotor() {
		return motor;
	}
	public static void setMotor(MotorWheels motor) {
		Robot.motor = motor;
	}
	public static UltrasonicSensor getUltrasonics() {
		return ultrasonics;
	}
	public void setUltrasonics(UltrasonicSensor ultrasonics) {
		this.ultrasonics = ultrasonics;
	}
	public EV3TouchSensor getTouch() {
		return touch;
	}
	public void setTouch(TouchSensor touch) {
		this.touch = touch;
	}
	public ColorSensor getColor() {
		return color;
	}
	public void setColor(ColorSensor color) {
		this.color = color;
	}
	
	public void stop() {
		motor.stop();
	}

	public boolean isPressed() {
		return touch.isPressed();
	}

	public static void setPaletpresent(boolean[] paletpresent) {
		Robot.paletpresent = paletpresent;
	}
	public boolean enMouvement() {
		return motor.enMouvement();
	}
}