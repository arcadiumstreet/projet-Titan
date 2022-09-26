package projet;

import lejos.hardware.BrickFinder;
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

import lejos.hardware.sensor.EV3TouchSensor;


public class FirstClass {
	
	
	public static void main(String[] args) {
		
		Robot pierrot = new Robot(MotorPort.B,MotorPort.C,MotorPort.A,SensorPort.S2,SensorPort.S3);
	
		
		pierrot.getUltrasonics().getDistance().fetchSample(pierrot.getUltrasonics().getSample(), 0);
		float dist = pierrot.getUltrasonics().getSample()[0];
		pierrot.getRightGear().rotate(360, true);
		int i = 0;
		while (Math.abs(pierrot.getUltrasonics().getSample()[0] - dist) < 0.1 && i < 30) {
			dist = pierrot.getUltrasonics().getSample()[0];
			
			i++;
		}
		pierrot.getRightGear().stop();
		if(i<30)
			pierrot.catchTarget((int) (pierrot.getUltrasonics().getSample()[0]));
		
		
		
		
		//pierrot.turn90Degres(pierrot.LEFT);
		//pierrot.turn90Degres(pierrot.RIGHT);
		//pierrot.turn180Degres(pierrot.LEFT);
		//pierrot.turn180Degres(pierrot.RIGHT);
		//pierrot.turn360Degres(pierrot.LEFT);
		//pierrot.turn360Degres(pierrot.RIGHT);
		
		//pierrot.moveCm(pierrot.FRONT, 50);
		
		//pierrot.moveCm(pierrot.BACK, 10);
 
		//pierrot.Research(pierrot.RIGHT);
		
		//pierrot.moveCm(pierrot.FRONT, 10);
		/*pierrot.getUltrasonics().getDistance().fetchSample(pierrot.getUltrasonics().getSample(), 0);
		if(pierrot.getUltrasonics().objectDetectedInRange(100))
			pierrot.catchTarget((int)(pierrot.getUltrasonics().getSample()[0] * 100));
		Delay.msDelay(1000);*/
		// attrapper un objet sans exception
		
		
		/*pierrot.getUltrasonics().getDistance().fetchSample(pierrot.getUltrasonics().getSample(), 0);
		float dist = pierrot.getUltrasonics().getSample()[0];
		int i = 0;
		while (Math.abs(pierrot.getUltrasonics().getSample()[0] - dist) < 0.3 && i < 30) {
			dist = pierrot.getUltrasonics().getSample()[0];
			pierrot.littleTurn(pierrot.RIGHT);
			i++;
			// pierrot.catchTarget((int)(pierrot.getUltrasonics().getSample()[0] * 100));
		}
		if (pierrot.getUltrasonics().getSample()[0] - dist < 0)
			pierrot.catchTarget((int) (pierrot.getUltrasonics().getSample()[0]));
		else {
			pierrot.littleTurn(pierrot.LEFT);
			pierrot.catchTarget((int) (pierrot.getUltrasonics().getSample()[0]));
		}*/
		//Delay.msDelay(1000);
		// tourne et attrapper un objet si detecté(non testé)
		
		
	}
	    private static void log(final String msg)
	    {
	        System.out.println("log>\t" + msg);
	        
	    }
		//leftMotor.backward();
		//rightMotor.backward();
		/*
		int i=0;
		leftMotor.setSpeed(900);
		rightMotor.setSpeed(900);
		pince.setSpeed(900);
		while(i<10000) {
		leftMotor.backward();
		rightMotor.backward();
		pince.backward();
		//pince2.forward();
		i++;}*/
		//pince.rotate(400);
		//rightMotor.rotate(180);
}