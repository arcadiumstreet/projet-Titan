package projet;

import lejos.hardware.motor.MindsensorsGlideWheelMRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.RegulatedMotor;

public class Robot {
	
	public static final int RIGHT = 1;
	public static final int FRONT = 1;
	public static final int LEFT = - 1;
	public static final int BACK = - 1;
	public static final int SPEED_PLIERS = 1000;
	public static final int SPEED_LEFTGEAR = 1000;
	public static final int SPEED_RIGHTGEAR = 1000;
	
	private static  RegulatedMotor leftGear;
	private static RegulatedMotor rightGear;
	private static RegulatedMotor pliers;
	
	private UltrasonicSensor ultrasonics;
	private EV3TouchSensor touch;
	
	private double angle;
	
	public Robot(Port leftGearPort,Port rightGearPort,Port pliersPort,Port ultrasonicsPort,Port touchPort)
	{
		this.leftGear = new MindsensorsGlideWheelMRegulatedMotor(leftGearPort);
		this.rightGear = new MindsensorsGlideWheelMRegulatedMotor(rightGearPort);
		this.pliers = new MindsensorsGlideWheelMRegulatedMotor(pliersPort);
		
		this.ultrasonics = new UltrasonicSensor(ultrasonicsPort) ;
		this.touch = new EV3TouchSensor(touchPort) ;
		
		leftGear.setSpeed(SPEED_LEFTGEAR);
		rightGear.setSpeed(SPEED_RIGHTGEAR);
		//leftGear.setAcceleration(SPEED_LEFTGEAR);
		//rightGear.setAcceleration(SPEED_RIGHTGEAR);
		pliers.setSpeed(SPEED_PLIERS);
		
		this.angle = 0;
		
	}
	
	public void updateAngle(double degree) {
		angle = (angle+degree)%360;
	}
	
	public static void catchTarget(int targetDistance)
	{
		openPliers();
		moveCm(FRONT,targetDistance + 3);
		closePliers();
		moveCm(BACK,targetDistance + 3);
	}
	// a faire d'autres prises 
	
	public static void moveCm(int direction,int distance)
	{
		leftGear.rotate(direction * distance * 21,true);
		rightGear.rotate(direction * distance * 21);
	}
	public void insertTurnDegres(int degree,int direction)
	{	
		leftGear.rotate(direction * degree,true);
		rightGear.rotate(direction * -degree);
		updateAngle(direction*degree*2.111);
	}
	
	public void littleTurn(int direction)
	{
		leftGear.rotate(direction * 20,true);
		rightGear.rotate(direction * -20);
		updateAngle(direction*9.5);
	}
	
	public void Research(int direction) {
		int i =0;
		while (i<20) {
			littleTurn(direction);
			i++;
		}
	}
	
	public void goal() {
		if (angle > 0) {
			insertTurnDegres((int)(-angle*2.111), RIGHT);
			updateAngle((int)(-angle*2.111));
		} else if (angle < 0) {
			insertTurnDegres((int)(-angle*2.111), LEFT);
			updateAngle((int)(-angle*2.111));
		}
		int i = 0;
		while(i < 500) {
			leftGear.forward();
			rightGear.forward();
			i++;
		}
	}
	
	public void turn90Degres(int direction)
	{	
		leftGear.rotate(direction * 190,true);
		rightGear.rotate(direction * (-190));
		updateAngle(direction*90);
	}
	public void turn180Degres(int direction)
	{	
		turn90Degres(2 * direction);
	}
	public void turn360Degres(int direction)
	{	
		turn180Degres(2 * direction);
	}
	
	public static  void openPliers()
	{
		pliers.rotate(500,true);
	}
	public static  void closePliers()
	{
		pliers.rotate(-500);
	}
	
	public static  void openPliers(int i)
	{
		pliers.rotate(i,true);
	}
	public static  void closePliers(int i)
	{
		pliers.rotate(-i,true);
	}
	
	

	public static RegulatedMotor getLeftGear() {
		return leftGear;
	}
	public static void setLeftGear(RegulatedMotor leftGear) {
		Robot.leftGear = leftGear;
	}
	public static RegulatedMotor getRightGear() {
		return rightGear;
	}
	public static void setRightGear(RegulatedMotor rightGear) {
		Robot.rightGear = rightGear;
	}
	public static RegulatedMotor getPliers() {
		return pliers;
	}
	public static void setPliers(RegulatedMotor pliers) {
		Robot.pliers = pliers;
	}
	public UltrasonicSensor getUltrasonics() {
		return ultrasonics;
	}
	public void setUltrasonics(UltrasonicSensor ultrasonics) {
		this.ultrasonics = ultrasonics;
	}
	public EV3TouchSensor getTouch() {
		return touch;
	}
	public void setTouch(EV3TouchSensor touch) {
		this.touch = touch;
	}

}
