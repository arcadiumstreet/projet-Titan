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
	public FirstClass() {
		
	}
	
	public static void main(String[] args) {
		
		Robot pierrot = new Robot(MotorPort.B,MotorPort.C,MotorPort.A,SensorPort.S2,SensorPort.S3);
		
		pierrot.turn90Degres(pierrot.LEFT);
		pierrot.turn90Degres(pierrot.RIGHT);
		pierrot.turn180Degres(pierrot.LEFT);
		pierrot.turn180Degres(pierrot.RIGHT);
		pierrot.turn360Degres(pierrot.LEFT);
		pierrot.turn360Degres(pierrot.RIGHT);
		
		pierrot.moveCm(pierrot.FRONT, 50);
		
		pierrot.moveCm(pierrot.BACK, 50);


		/*
		pierrot.getUltrasonics().getDistance().fetchSample(pierrot.getUltrasonics().getSample(), 0);
		if(pierrot.getUltrasonics().objectDetectedInRange(100))
			pierrot.catchTarget((int)(pierrot.getUltrasonics().getSample()[0] * 100));
		Delay.msDelay(1000);
		*/
		/*	---------- ULTRASONICS SENSOR ---------- */
		/*
		SensorModes ultrasonic = new EV3UltrasonicSensor(SensorPort.S2);
		SampleProvider distance= ultrasonic.getMode("Distance");
		float[] sample = new float[distance.sampleSize()];

		
		distance.fetchSample(sample, 0);
		System.out.println(sample[0]);


		if(sample[0] < 1)
			catchTarget(leftMotor,rightMotor, pince,(int)(sample[0] * 100));
		Delay.msDelay(1000);
		*/
		/*
		int i = 0;
		while(i<5000) 
		{
			//moveForward(leftMotor,rightMotor);
			distance.fetchSample(sample, 0);
			System.out.println(sample[0]);
			i++;

			if(sample[0] < 0.5)
			{	
				turn90(leftMotor,rightMotor,RIGHT);
				log("!!!!!	OBSTACLE !!!!!");
				
			}
		
		}
		*/
		
		/*	---------- TOUCH SENSOR ---------- */
		/*
		EV3TouchSensor touch = new EV3TouchSensor(SensorPort.S3);
		int i = 0;
		float[] sample = new float[1];
		log("START");
		openPliers(pince);
		
		while (sample[0] == 0)
		{
			moveForward(leftMotor,rightMotor);
			touch.fetchSample(sample, 0);
			System.out.println(sample[0]);
	       i++;
		}
		closePliers(pince);
		log("END");
	    */    
	}
	    private static void log(String msg)
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
