package projet;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.utility.Delay;

public class Agent {

	private Actuators act;
	private Sensors sens;
	private double angle;
	
	public Agent(Port leftGearPort,Port rightGearPort,Port pliersPort,Port ultrasonicsPort,Port touchPort) {
		// TODO Auto-generated constructor stub
		act= new Actuators(leftGearPort,rightGearPort,pliersPort);
		sens=new Sensors(ultrasonicsPort,touchPort);
		angle=0;
	}
	
	public void updateAngle(double degree) {
		angle = (angle+degree)%360;
	}
	
	public void catchTarget(int targetDistance){
		boolean b=true;
		act.openPliers();
		act.moveCmLibre(act.FRONT,targetDistance + 3);
		while(b) {
			if(sens.getDistance()<=0.25) {
				act.stopWheels();
				b=false;
			}
		}
		act.closePliers();
		//act.moveCmBloc(BACK,targetDistance + 3);
	}
	// a faire d'autres prises 
	
	public void research() {
		float dis=1;
		while(dis>0.5){
			dis = sens.getDistance();
			act.getLeftGear().forward();
			act.getRightGear().backward();
			//System.out.print(sens.getDistance();
			}
		act.stopWheels();
		
		act.getLeftGear().setSpeed(act.SPEED_LEFTGEAR);
		act.getRightGear().setSpeed(act.SPEED_RIGHTGEAR);
		catchTarget((int)(100*dis));
	}
	
	public void goal() {
		if (angle > 0) {
			act.insertTurnDegresBloc((int)(-angle*2.111), act.RIGHT);
			updateAngle((int)(-angle*2.111));
		} else if (angle < 0) {
			act.insertTurnDegresBloc((int)(-angle*2.111), act.LEFT);
			updateAngle((int)(-angle*2.111));
		}
		int i = 0;
		while(i < 500) {
			act.getLeftGear().forward();
			act.getRightGear().forward();
			i++;
		}
	}
	

	public static void main(String[] args) {
		// TODO Auto-generated method stub

		Agent pierrot= new Agent(MotorPort.B,MotorPort.C,MotorPort.A,SensorPort.S2,SensorPort.S3);
		
		Delay.msDelay(5000);
	}

}
