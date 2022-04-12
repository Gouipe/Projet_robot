package cedric;

import java.util.ArrayList;
import java.util.ListIterator;

import lejos.hardware.Button;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.MovePilot;

public class MonPilot extends MovePilot {
	
	private final int PINCES_FERMEES = 0;
	private final int PINCES_OUVERTES = 1;

	private int etatPinces; //indique si les pinces sont ouvertes ou ferm�es
	
	private RegulatedMotor pinces;
	

	/**
	 * @param wheelDiameter
	 * @param trackWidth
	 * @param leftMotor
	 * @param rightMotor
	 * @param pinces
	 * Cr�e un pilote pour un robot avec pinces qui doivent �tre ferm�es � l'instanciation.
	 */
	public MonPilot(double wheelDiameter, double trackWidth, RegulatedMotor leftMotor, RegulatedMotor rightMotor, RegulatedMotor pinces) {
		super(wheelDiameter, trackWidth, leftMotor, rightMotor);
		this.pinces = pinces;
		etatPinces = PINCES_FERMEES; // pinces doivent �tre ferm�es au d�but
	}
	
	/**
	 * Verifie si les pinces sont ouvertes, auquel cas ferme les pinces et change etatPinces.
	 */
	public void fermePinces() {
		//TODO
	}
	
	/**
	 * Verifie si les pinces sont ferm�es, auquel cas ouvre les pinces et change etatPinces.
	 */
	public void ouvrePinces() {
		//TODO
	}
	
	

}
