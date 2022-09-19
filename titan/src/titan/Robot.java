package titan;

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
	
	public Robot(Port leftGearPort,Port rightGearPort,Port pliersPort,Port ultrasonicsPort,Port touchPort)
	{
		this.leftGear = new MindsensorsGlideWheelMRegulatedMotor(leftGearPort);
		this.rightGear = new MindsensorsGlideWheelMRegulatedMotor(rightGearPort);
		this.pliers = new MindsensorsGlideWheelMRegulatedMotor(pliersPort);
		
		this.ultrasonics = new UltrasonicSensor(ultrasonicsPort) ;
		this.touch = new EV3TouchSensor(touchPort) ;
		
		leftGear.setSpeed(SPEED_LEFTGEAR);
		rightGear.setSpeed(SPEED_RIGHTGEAR);
		leftGear.setAcceleration(SPEED_LEFTGEAR);
		rightGear.setAcceleration(SPEED_RIGHTGEAR);
		pliers.setSpeed(SPEED_PLIERS);
		
		
	}
	public static void catchTarget(int targetDistance)
	{
		openPliers();
		moveCm(FRONT,targetDistance + 3);
		closePliers();
		moveCm(BACK,targetDistance + 3);
	}
	
	public static void moveCm(int direction,int distance)
	{
		leftGear.rotate(direction * distance * 21,true);
		rightGear.rotate(direction * distance * 21);
	}
	public static void turn90Degres(int direction)
	{	
		leftGear.rotate(direction * 190,true);
		rightGear.rotate(direction * (-190));
	}
	public static void turn180Degres(int direction)
	{	
		turn90Degres(2 * direction);
	}
	public static void turn360Degres(int direction)
	{	
		turn180Degres(2 * direction);
	}
	
	public static  void openPliers()
	{
		pliers.rotate(500,true);
	}
	public static  void closePliers()
	{
		pliers.rotate(-500,true);
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
