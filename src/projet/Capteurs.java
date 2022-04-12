package projet;

import lejos.hardware.sensor.EV3UltrasonicSensor;

public class Capteurs {
	EV3UltrasonicSensor soundSensor;
	TouchSensor touchSensor;
	
	public Capteurs(EV3UltrasonicSensor soundSensor, TouchSensor touchSensor) {
		this.soundSensor = soundSensor;
		this.touchSensor = touchSensor;
	}

}
