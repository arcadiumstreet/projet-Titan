package projet;

import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.utility.Delay;

public class test {

	
	public static void main(String[] args) {
		int placement=1;
		Robot p = new Robot(MotorPort.B,MotorPort.C,MotorPort.A,SensorPort.S2,SensorPort.S3,placement);
		cerveau d = new cerveau();
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
		
	//	p.getPinces().reglagepinces(-100);
		
		//p.fermerPinces(false);
		//
		//p.getPinces().reglagepinces(100);
		//d.strategie1(p,1);
		d.strategie2(p,2,3,9);
		
		 
		
	}
}
