package projet;

import java.util.ArrayList;
import java.util.ListIterator;

import lejos.hardware.Button;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;

public class MonPilot extends MovePilot {

	// indique si les pinces sont ouvertes ou ferm�es. C'est une simple
	// s�curit� suppl�mentaire, pour ne pas appeler la m�me m�thode 2 fois de suite
	private boolean ouvertes; 
						
	EV3MediumRegulatedMotor pinces;
	private double angleAbsoluCourant = 0.0;


	/**
	 * @param wheelDiameter
	 * @param trackWidth
	 * @param leftMotor
	 * @param rightMotor
	 * @param pinces        Cr�e un pilote pour un robot avec pinces qui doivent
	 *                      �tre ferm�es � l'instanciation.
	 */
	public MonPilot(double wheelDiameter, double trackWidth, RegulatedMotor leftMotor, RegulatedMotor rightMotor,
			EV3MediumRegulatedMotor pinces) {
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
			Delay.msDelay(2000); 
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
			Delay.msDelay(2000);
			pinces.stop();
		//}
		ouvertes = true;
	}
	
	/*
	public void rotate(double angle, boolean immediateReturn) {
		super.rotate(angle, immediateReturn);
		angleAbsoluCourant = (angleAbsoluCourant + angle) % 360;
		System.out.println("rotate1:" + angleAbsoluCourant);
		//Button.ENTER.waitForPressAndRelease();
	}
	*/
	
	public void rotate(double angle) {
		super.rotate(angle);
		angleAbsoluCourant = (angleAbsoluCourant + angle) % 360;
		//Button.ENTER.waitForPressAndRelease();
	}
	
	
	public void rotateWhileHoldingPalet(double angle, boolean immediateReturn) {
		super.rotate(angle, immediateReturn);
		angleAbsoluCourant = (angle * 0.95 + angleAbsoluCourant) % 360; 
		//on enl�ve quelques % de l'angle th�orique
		// car le robot tourne un peu moins que pr�vu quand il tient un palet
		//Button.ENTER.waitForPressAndRelease();
	}
	
	public void rotateToGoal() {
		rotateWhileHoldingPalet(-angleAbsoluCourant, true);
	}

}
