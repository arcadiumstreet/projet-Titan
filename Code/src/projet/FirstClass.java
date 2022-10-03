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
import java.math.*;
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
		
		
		
		
		pierrot.moveCm(pierrot.FRONT, 100);
		
		/*pierrot.getLeftGear().setSpeed(500);
		pierrot.getRightGear().setSpeed(500);
		pierrot.turn180Degres(pierrot.RIGHT);
		pierrot.turn90Degres(pierrot.RIGHT);
		pierrot.turn180Degres(pierrot.RIGHT);
		pierrot.goal();*/
		
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