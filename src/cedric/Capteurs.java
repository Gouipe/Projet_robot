package cedric;

import lejos.hardware.sensor.EV3UltrasonicSensor;

public class Capteurs {
	EV3UltrasonicSensor soundSensor;
	
	public Capteurs(EV3UltrasonicSensor soundSensor) {
		this.soundSensor = soundSensor;
	}

}
