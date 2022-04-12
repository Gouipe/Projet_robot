package cedric;

import java.util.ArrayList;
import java.util.ListIterator;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.MovePilot;
//Button.ENTER.waitForPressAndRelease();

public class Nettoyage {

	final private double DISTANCE_MIN_PALET = 31.0; // distance pour laquelle le palet n'est plus d�tect�

	final private int SCAN_PALETS = 0;
	final private int ORIENTER_PALET=1;
	
	MonPilot robot;
	Capteurs capteurs;
	int etat; // Etat courant de l'automate

	public Nettoyage(MonPilot robot, Capteurs capteurs) {
		this.robot = robot;
		this.capteurs = capteurs;
		etat = SCAN_PALETS; //au d�but, on va chercher la position des palets
	}

	public void run() {
		//while non fini
			switch(etat) {
				case SCAN_PALETS :
					// on ferme les pinces (il faut v�rifier si les pinces sont d�j� ferm�es)
					robot.fermePinces();
					
					// on cherches les palets potentiels
					scanPalet();
					
					//on modifie l'�tat
					//etat = ;
					
					break;
					
				// tant que la distance mesur�e est > � 31cm (- quelques cm par s�curit�), cedric avance
		
				// Si la distance mesur�e passe de 31 cm � 0 : on sait qu'il y a un palet, on le r�cup�re
				//attrapePalet();
				case ORIENTER_PALET:
					
					break;
			}
	}

	/**
	 * Attrape un palet. A utiliser quand le palet n'est plus d�tect� alors qu'on
	 * avance vers lui
	 */
	public void attrapePalet() {
		// TODO
	}

	/**
	 * Range le palet une fois q'il est attrap�
	 */
	public void rangePalet() {
		// TODO
	}

	/**
	 * Scanne alentour pour chercher des palets. S'oriente alors vers le palet
	 * potentiel le plus proche
	 */
	public void scanPalet() {
		SampleProvider sampleProvider = capteurs.soundSensor.getDistanceMode();

		// cedric tourne sur lui meme sur 360 degres et r�cup�re tout du long les
		// mesures dans l'arraylist distances
		int nbMesures = 0;
		ArrayList<Float> distances = new ArrayList<Float>();
		robot.rotate(360, true);
		while (robot.isMoving()) {
			float[] sample = new float[sampleProvider.sampleSize()];
			sampleProvider.fetchSample(sample, 0);
			distances.add(sample[0]);
			nbMesures++;
		}
		double degreParMesure = 360.0 / nbMesures; //degre parcouru entre deux mesures
		
		// TESTS///////		
		System.out.println("nbMesures : " + nbMesures);
		System.out.println("taille distance : " + distances.size());
		System.out.println("degreParMesure : " + degreParMesure);
		Button.ENTER.waitForPressAndRelease();

		System.out.println("0 : " + distances.get(0));
		Button.ENTER.waitForPressAndRelease();

		System.out.println("90 : " + distances.get(distances.size() / 4));
		Button.ENTER.waitForPressAndRelease();

		System.out.println("180 : " + distances.get(distances.size() / 2));
		Button.ENTER.waitForPressAndRelease();

		System.out.println("270 : " + distances.get(distances.size() * 3 / 4));
		Button.ENTER.waitForPressAndRelease();
		///////////

		/***On r�cup�re dans la variable min la plus petite distance qui peut correspondre � un palet****/
		float min = distances.get(0);// initialisation
		int quantiemeMesure = 0; // nombre de mesures faites avant la mesure min
		ListIterator<Float> it = distances.listIterator();
		float courant;
		while (it.hasNext()) {
			courant = it.next();
			if (courant < min && courant > DISTANCE_MIN_PALET) { // courant est une distance qui peut correspondre � un
																	// palet
				min = courant;
				quantiemeMesure = it.nextIndex(); // l'index corresond au nombre de mesures / � la combientieme mesure
			}
		}
		/*************************************************************************************************/
		
		// cedric rotate vers la direction qui correpond � la distance min
		robot.rotate(degreParMesure * quantiemeMesure); // (degre entre deux mesures) * nbmesures => degre vers min
	}
}
