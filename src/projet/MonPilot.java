package projet;

import java.util.ArrayList;
import java.util.ListIterator;

import lejos.hardware.Button;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;

public class MonPilot extends MovePilot {

	// indique si les pinces sont ouvertes ou ferm�es. C'est une simple
	// s�curit� suppl�mentaire, pour ne pas appeler la m�me m�thode 2 fois de suite
	private boolean ouvertes; 
						
	RegulatedMotor pinces;

	/**
	 * @param wheelDiameter
	 * @param trackWidth
	 * @param leftMotor
	 * @param rightMotor
	 * @param pinces        Cr�e un pilote pour un robot avec pinces qui doivent
	 *                      �tre ferm�es � l'instanciation.
	 */
	public MonPilot(double wheelDiameter, double trackWidth, RegulatedMotor leftMotor, RegulatedMotor rightMotor,
			RegulatedMotor pinces) {
		super(wheelDiameter, trackWidth, leftMotor, rightMotor);
		this.pinces = pinces;
		ouvertes = false; // pinces doivent �tre ferm�es au d�but
	}

	/**
	 * Verifie si les pinces sont ouvertes, auquel cas ferme les pinces et change
	 * l'attribut ouvertes
	 */
	public void fermePinces() {
		//if (ouvertes) {
			pinces.backward();
			Delay.msDelay(1666);
			pinces.stop();
		//}
		ouvertes = false;
	}

	/**
	 * Verifie si les pinces sont ferm�es, auquel cas ouvre les pinces et change
	 * etatPinces.
	 */
	public void ouvrePinces() {
		//if (!ouvertes) {
			pinces.forward();
			Delay.msDelay(1666);
			pinces.stop();
		//}
		ouvertes = true;
	}

}
