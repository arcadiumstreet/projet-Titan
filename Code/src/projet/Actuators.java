package projet;


import lejos.robotics.RegulatedMotor;
import lejos.hardware.motor.MindsensorsGlideWheelMRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;


public class Actuators {

	public static final int RIGHT = 1;
	public static final int FRONT = 1;
	public static final int LEFT = - 1;
	public static final int BACK = - 1;
	public static final int SPEED_PLIERS = 1000;
	public static final int SPEED_LEFTGEAR = 1000;
	public static final int SPEED_RIGHTGEAR = 1000;
	public static final int MAX_SPEED=1000;
	public static final int MID_SPEED=500;
	public static final int MIN_SPEED=250;
	
	private  RegulatedMotor leftGear;
	private RegulatedMotor rightGear;
	private RegulatedMotor pliers;
	
	
	
	public Actuators(Port leftGearPort,Port rightGearPort,Port pliersPort) {
		// TODO Auto-generated constructor stub
		leftGear = new MindsensorsGlideWheelMRegulatedMotor(leftGearPort);
		rightGear = new MindsensorsGlideWheelMRegulatedMotor(rightGearPort);
		pliers = new MindsensorsGlideWheelMRegulatedMotor(pliersPort);
		
		leftGear.setSpeed(MAX_SPEED);
		rightGear.setSpeed(MAX_SPEED);
		pliers.setSpeed(MAX_SPEED);

	}
	
	public void moveCmBloc(int direction,int distance){
		leftGear.rotate(direction * distance * 21,true);
		rightGear.rotate(direction * distance * 21,false);
	}
	
	public void moveCmLibre(int direction,int distance){
		leftGear.rotate(direction * distance * 21,true);
		rightGear.rotate(direction * distance * 21,true);
	}
	
	public void insertTurnDegresBloc(int degree,int direction){	
		leftGear.rotate(direction * degree,true);
		rightGear.rotate(direction * -degree,false);	
	}
	
	public void insertTurnDegresLibre(int degree,int direction){	
		leftGear.rotate(direction * degree,true);
		rightGear.rotate(direction * -degree,true);	
	}
	
	public void stopRight() {
		rightGear.stop();
	}
	
	public void stopLeft() {
		leftGear.stop();
	}
	
	public void stopPliers() {
		pliers.stop();
	}
	
	public void stopWheels() {
		stopRight();stopLeft();
	}
	
	public void stopAll() {
		stopWheels();stopPliers();
	}
	
	public RegulatedMotor getLeftGear(){
		return leftGear;
	}
	
	public void setLeftGear(RegulatedMotor newLeftGear) {
		leftGear = newLeftGear;
	}
	
	public RegulatedMotor getRightGear(){
		return rightGear;
	}
	
	public void setRightGear(RegulatedMotor newRightGear) {
		rightGear = newRightGear;
	}
	
	public RegulatedMotor getPliers(){
		return pliers;
	}
	
	public void setPliers(RegulatedMotor newPliers) {
		pliers = newPliers;
	}
	
	public void openPliers(){
		openPliers(700);
	}
	
	public void closePliers(){
		closePliers(700);
	}
	
	public void openPliers(int i){
		pliers.rotate(i);
	}
	
	public void closePliers(int i){
		pliers.rotate(-i);
	}
	
	

	public static void main(String[] args) {
		// TODO Auto-generated method stub

	}

}
