package projet;

import lejos.hardware.motor.MindsensorsGlideWheelMRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.robotics.Color;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;

public class Robot {
	
	public static final int RIGHT = 1;
	public static final int FRONT = 1;
	public static final int LEFT = - 1;
	public static final int BACK = - 1;
	public static final int SPEED_PLIERS = 1000;
	public static final int SPEED_LEFTGEAR = 1000;
	public static final int SPEED_RIGHTGEAR = 1000;
	public static final int TURN_SPEED = 500;
	
	private static RegulatedMotor leftGear;
	private static RegulatedMotor rightGear;
	private static RegulatedMotor pliers;
	
	private static UltrasonicSensor ultrasonics;
	private static EV3TouchSensor touch;
	private static ColorSensor color;
	
	private double angle;
	
	public Robot(Port leftGearPort,Port rightGearPort,Port pliersPort,Port ultrasonicsPort,Port touchPort,Port colorport){
		this.leftGear = new MindsensorsGlideWheelMRegulatedMotor(leftGearPort);
		this.rightGear = new MindsensorsGlideWheelMRegulatedMotor(rightGearPort);
		this.pliers = new MindsensorsGlideWheelMRegulatedMotor(pliersPort);
		
		this.ultrasonics = new UltrasonicSensor(ultrasonicsPort) ;
		this.touch = new EV3TouchSensor(touchPort) ;
		this.color = new ColorSensor("S1");
		
		leftGear.setSpeed(SPEED_LEFTGEAR);
		rightGear.setSpeed(SPEED_RIGHTGEAR);
		pliers.setSpeed(SPEED_PLIERS);
		this.angle = 0;
	} 
 
	public String color() {
        Color rgb = ColorSensor.getColorIn();
        return ColorSensor.color_String(rgb.getRed(), rgb.getGreen(), rgb.getBlue());
                //getColor().toString(getColor().getRed(),getColor().getGreen(),getColor().getColor().getBlue());
    }
	
	public void allerjusqua(String couleur) {
		leftGear.forward();
		rightGear.forward();
	    Color rgb = ColorSensor.getColorIn();
	    while( ColorSensor.color_String(rgb.getRed(), rgb.getGreen(), rgb.getBlue()) != couleur) {
	    	rgb = ColorSensor.getColorIn();
	    }
	    leftGear.stop();
	    rightGear.stop();
	}
	
	public void updateAngle(double degree) {
		angle = (angle+degree)%360;
	}
	
	public static void catchTarget(int targetDistance)
	{
		leftGear.setSpeed(SPEED_LEFTGEAR);
		rightGear.setSpeed(SPEED_RIGHTGEAR);
		openPliers();
		moveCm(FRONT,targetDistance + 3);
		closePliers();
		//moveCm(BACK,targetDistance + 3);
	}
	// a faire d'autres prises 
	
	public static void moveCm(int direction,int distance)
	{
		leftGear.setSpeed(SPEED_LEFTGEAR);
		rightGear.setSpeed(SPEED_RIGHTGEAR);
		leftGear.rotate(direction * distance * 21,true);
		rightGear.rotate(direction * distance * 21);
	}
	public void insertTurnDegres(int degree,int direction)
	{	
		leftGear.setSpeed(TURN_SPEED);
		rightGear.setSpeed(TURN_SPEED);
		leftGear.rotate(direction * degree,true);
		rightGear.rotate(direction * -degree);
		updateAngle(direction*degree/2.2);
	}
	
	public void research() {
		int i =0;
		float dis=1;
		leftGear.setSpeed(TURN_SPEED);
		rightGear.setSpeed(TURN_SPEED);
		leftGear.forward();
		rightGear.backward();
		while(/*i < 975*/dis>0.5){
			getUltrasonics().getDistance().fetchSample(getUltrasonics().getSample(), 0);
			dis = getUltrasonics().getSample0();
			//i++;
			//System.out.print(angle);
			//System.out.print(dis);
			}
		leftGear.stop();
		rightGear.stop();
		
		catchTarget((int)(100*dis));
	}
	
	public void goal() {
		if (angle > 180) {
			insertTurnDegres((int)((360-angle)*2.2), LEFT);
		} else {
			insertTurnDegres((int)(angle*2.2), LEFT);
		}
		leftGear.setSpeed(SPEED_LEFTGEAR);
		rightGear.setSpeed(SPEED_RIGHTGEAR);
		leftGear.forward();
		rightGear.forward();
		int i = 0;
		while(i < 500) { 
			i++;
		}
		leftGear.stop();
		rightGear.stop();

		//ultrasonics.arrete();
		openPliers();
		moveCm(BACK, 50);
		closePliers();
	}
	
	public void turn90Degres(int direction)
	{
		leftGear.setSpeed(TURN_SPEED);
		rightGear.setSpeed(TURN_SPEED);
		leftGear.rotate(direction * 198,true);
		rightGear.rotate(direction * (-198));
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
		pliers.rotate(700);
	}
	public static  void closePliers()
	{
		pliers.rotate(-700);
	}
	public static  void openPliers(int i)
	{
		pliers.rotate(i);
	}
	public static  void closePliers(int i)
	{
		pliers.rotate(-i);
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
	public static UltrasonicSensor getUltrasonics() {
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
	public ColorSensor getColor() {
		return color;
	}
	public void setColor(ColorSensor color) {
		this.color = color;
	}
}