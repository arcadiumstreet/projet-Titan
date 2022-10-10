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


public class FirstClass {
	
	
	public static void main(String[] args) {
		
		Robot pierrot = new Robot(MotorPort.B,MotorPort.C,MotorPort.A,SensorPort.S2,SensorPort.S3,SensorPort.S1);

		for (int i = 0; i < 10; i++) {
			pierrot.insertTurnDegres((int)(Math.random()*360),pierrot.RIGHT);
		}
		pierrot.goal();
	
		
		/*pierrot.getUltrasonics().getDistance().fetchSample(pierrot.getUltrasonics().getSample(), 0);
		float dist = pierrot.getUltrasonics().getSample()[0];
		pierrot.getRightGear().rotate(360, true);
		int i = 0;
		while (Math.abs(pierrot.getUltrasonics().getSample()[0] - dist) < 0.1 && i < 30) {
			dist = pierrot.getUltrasonics().getSample()[0];
			
			i++;
		}
		pierrot.getRightGear().stop();
		if(i<30)
			pierrot.catchTarget((int) (pierrot.getUltrasonics().getSample()[0]));*/

		//pierrot.moveCm(pierrot.FRONT, 100);
		//pierrot.closePliers();
		//pierrot.turn180Degres(pierrot.LEFT);
		//pierrot.turn360Degres(pierrot.RIGHT);
		//pierrot.goal();
		//pierrot.turn90Degres(pierrot.RIGHT);
		//pierrot.goal();
		/*pierrot.getUltrasonics().getDistance().fetchSample(pierrot.getUltrasonics().getSample(), 0);
		if(pierrot.getUltrasonics().objectDetectedInRange(100))
			pierrot.catchTarget((int)(pierrot.getUltrasonics().getSample()[0] * 100));
		Delay.msDelay(1000);*/
		// attrapper un objet sans exception
		//pierrot.openPliers(); 
		  
		 pierrot.allerjusqua("BLANC");
		 //pierrot.research();
		/*boolean again =true;
		while (again) {
		 System.out.println("\nPress enter to detect a color...");
			Button.ENTER.waitForPressAndRelease();
			System.out.println("la couleur est "+pierrot.color());
			Delay.msDelay(500);
			if(Button.ESCAPE.isDown()) {
				again = false;
			}
		 

		//pierrot.closePliers(300);
		}*/
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