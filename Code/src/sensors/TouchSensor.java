package sensors;

import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3TouchSensor;

public class TouchSensor extends EV3TouchSensor{

	/**
	 * initiallise une instance de TouchSensor à partir de l' EV3TouchSensor branché sur port
	 * 
	 * @param port le port de branchement de l'EV3TouchSensor
	 */
	public TouchSensor(Port port) {
		super(port);
	}
	
	/**
	 * indique si le détecteur de touché est activé
	 * 
	 * @return true si le detecteur de touché est activé, false sinon
	 */
	public boolean isPressed() {
        float[] sample = new float[1];
        fetchSample(sample, 0);

        return sample[0] != 0;
    }
	
}