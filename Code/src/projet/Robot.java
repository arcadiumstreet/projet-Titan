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
	
	public Robot(Port leftGearPort,Port rightGearPort,Port pliersPort,Port ultrasonicsPort,Port touchPort,Port colorport,int i){

		motor = new MotorWheels(leftGearPort,rightGearPort, i);
		pinces = new Pinces(pliersPort);
		
		ultrasonics = new UltrasonicSensor(ultrasonicsPort) ;
		touch = new TouchSensor(touchPort) ;
		color = new ColorSensor("S1");

	} 
	
	public void test() {
	}
	
	
	public String colorint() {
        int [] e = color.getcolorint();
        String s = Arrays.toString(e) ;
         return s ;
	}
	
	public String color() {
        Color rgb = ColorSensor.getColor();
        return ColorSensor.color_String(rgb.getRed(), rgb.getGreen(), rgb.getBlue());
    }
	
	//initialiser apres longueur aux max 
	public void allerjusqua(String couleur) {
		motor.forward();
	    Color rgb = ColorSensor.getColor();
	    while( ColorSensor.color_String(rgb.getRed(), rgb.getGreen(), rgb.getBlue()) != couleur) {
	    	rgb = ColorSensor.getColor();
	    }
	   motor.stop();
	}
	
	public void forward() {
		motor.forward();
	}

	public void forward(double d) {
		motor.forward(d);
	}
	
	public void backward(double d) {
		motor.backward(d);
	}
	
	public void forward(double d,boolean b) {
		motor.forward(d,b);
	}
	
	
	
	public void demitour() {
		motor.rotate(180);
	}
	public void rotate(double d) {
		motor.rotate(d);
	}
	public void rotate(double d,boolean b) {
		motor.rotate(d,b);
	}
	public void rotateneg(double d,boolean b) {
		motor.rotate(-d,b);
	}
	public void lap() {
		motor.rotate();
	}
	public void boussole_a_0() {
		motor.boussole_a_0();
	}
	
	public void allera(double x, double y) {
		motor.goTo(x, y);        
		//motor.moveToCoordinates(x,y);
	}
	public boolean catchTarget(int targetDistance){
		ouvrirPinces();
		motor.forward(targetDistance + 3);
		if (touch.isPressed()) {
			fermerPinces();
			return true;
		} else {
			fermerPinces();
			return false;
		}
	}
	
	public double research() {
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
		motor.mettreAJourBoussole(angle);
		return dis ;
	}
	
	public void goal() {
		motor.boussole_a_0();
		allerjusqua("BLANC");
		erreurs_boussole();
		ouvrirPinces();
		motor.backward(200);
		fermerPinces();
	}
	
	public void erreurs_boussole() {
        motor.rotate(30,false);
        motor.rotate(-60,true);
        double min = 100;
        double angle_trouver = 0;
        while(motor.enMouvement()) {
        	getUltrasonics().getDistance().fetchSample(getUltrasonics().getSample(), 0);
            double valeur_en_cours = getUltrasonics().getSample0();
            if(valeur_en_cours<min) {
                min=valeur_en_cours;
                angle_trouver = motor.angle();
            }
            Delay.msDelay(3);
        }
        System.out.println("angle trouver "+angle_trouver);
        System.out.println("valeurs min"+min);
        motor.rotate(60+angle_trouver, false);
        motor.setBoussole(0);
    }

	public void ouvrirPinces() {
		pinces.ouvrir();
	}
	public void fermerPinces() {
		pinces.fermer();
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
		// TODO Auto-generated method stub
		return touch.isPressed();
	}
	
}