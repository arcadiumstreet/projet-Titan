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
		
		boolean again =true;
		while (again) {
		 System.out.println("\nPress enter to detect a color...");
			Button.ENTER.waitForPressAndRelease();
			System.out.println("la couleur est "+pierrot.color());
			Delay.msDelay(500);
			if(Button.ESCAPE.isDown()) {
				again = false;
			}
		}
	}
	    private static void log(final String msg)
	    {
	        System.out.println("log>\t" + msg);
	        
	    }
}