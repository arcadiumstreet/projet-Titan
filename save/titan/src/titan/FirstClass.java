package titan;
import lejos.hardware.BrickFinder;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.motor.MindsensorsGlideWheelMRegulatedMotor;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.EncoderMotor;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;

public class FirstClass {
	public FirstClass() {
		
	}
	public static void main(String[] args) {
		RegulatedMotor leftMotor = new MindsensorsGlideWheelMRegulatedMotor(MotorPort.B);
		RegulatedMotor rightMotor = new MindsensorsGlideWheelMRegulatedMotor(MotorPort.C);
		//leftMotor.rotate(180);
		int i=0;
		leftMotor.setSpeed(900);
		rightMotor.setSpeed(900);
		while(i<1000
				) {
		leftMotor.backward();
			leftMotor.backward();
		rightMotor.backward();
		i++;}
		//rightMotor.rotate(180);
		
		
	}

}
