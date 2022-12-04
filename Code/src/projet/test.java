package projet;

import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.utility.Delay;

public class test {

	
	public static void main(String[] args) {
		int placement=2;
		Robot p = new Robot(MotorPort.B,MotorPort.C,MotorPort.A,SensorPort.S4,SensorPort.S3,placement);
		/*	
		 //test les couleurs 
			boolean again =true;
			while (again) {
			 System.out.println("\nPress enter to detect a color...");
				Button.ENTER.waitForPressAndRelease();
				//System.out.println(pierrot.colorint());
				System.out.println("la couleur est "+pierrot.color());
				Delay.msDelay(1000);
				if(Button.ESCAPE.isDown()) {
					again = false;}}
					*/
		//p.getPinces().reglagepinces(100);
		
	
		//Homologation
		p.catchTarget(1200);
		p.goal(false);
		//
		p.allerjusqua("BLANC");
		
		
}
}