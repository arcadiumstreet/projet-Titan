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
	public static final int TURN_SPEED = 500;

	private static Pinces pinces ;
	private static MotorWheels motor;
	
	private static UltrasonicSensor ultrasonics;
	private static TouchSensor touch; 
	private static ColorSensor color;
	
	public Robot(Port leftGearPort,Port rightGearPort,Port pliersPort,Port ultrasonicsPort,Port touchPort,Port colorport){
		motor = new MotorWheels(leftGearPort,rightGearPort);
		pinces = new Pinces(pliersPort);
		
		ultrasonics = new UltrasonicSensor(ultrasonicsPort) ;
		touch = new TouchSensor(touchPort) ;
		color = new ColorSensor("S1");
	} 
	
	public void test() {
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
		//Delay.msDelay(10000);
		motor.mettreAJourBoussole(angle);
		motor.boussole_a_0();
		allerjusqua("BLANC");
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
	
	public void avancer(int i) {
		motor.forward(i);
	}
	public void reculer(int i) {
		motor.backward(i);
	}
	public void demitour() {
		motor.rotate(180);
	}
	public void rotate(double d) {
		motor.rotate(d);
	}
	public void boussole_a_0() {
		motor.boussole_a_0();
	}
	
	public boolean catchTarget(int targetDistance){
		ouvrirPinces();
		motor.forward(targetDistance + 3);
		fermerPinces();
		if (touch.isPressed()) {
			return true;
		} else {
			return false;
		}
	}
	
	public void research() {
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
	}
	
	public void goal() {
		motor.boussole_a_0();
		allerjusqua("BLANC");
		ouvrirPinces();
		motor.backward(200);
		fermerPinces();
	}
	public static void ouvrirPinces() {
		pinces.ouvrir();
	}
	public static void fermerPinces() {
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
}