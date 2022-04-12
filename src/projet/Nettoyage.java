package projet;

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
import lejos.utility.Delay;

public class Nettoyage {

	final private double DISTANCE_MIN_PALET = 0.321; // distance pour laquelle le palet n'est plus détecté

	final private int SCAN_PALETS = 0;
	final private int DIRIGER_VERS_PALET = 1;
	final private int ATTRAPER_PALET = 2;
	final private int RANGER_PALET_ATTRAPE = 3;

	MonPilot robot;
	Capteurs capteurs;
	int etat; // Etat courant de l'automate

	public Nettoyage(MonPilot robot, Capteurs capteurs) {
		this.robot = robot;
		this.capteurs = capteurs;
		etat = SCAN_PALETS; // au début, on va chercher la position des palets
	}

	public void run() {
		// while non fini
		switch (etat) {
		case SCAN_PALETS:
			// on ferme les pinces (il faut vérifier si les pinces sont déjà fermées)
			robot.fermePinces();

			// on cherches les palets potentiels
			if (scanPalet() == 0)
				etat = DIRIGER_VERS_PALET;
			break;

		case DIRIGER_VERS_PALET:
			dirigerVersPalet();
			break;

		case ATTRAPER_PALET:
			if (attrapePalet() == 0) // si on attrape bien un palet, on va le ranger
				etat = RANGER_PALET_ATTRAPE;
			else // sinon on cherche un nouveau palet
				etat = SCAN_PALETS;
			break;

		case RANGER_PALET_ATTRAPE:
			rangePalet();
			// une fois que le palet est posé, on cherche un nouveau palet
			etat = SCAN_PALETS;
			break;
		}
	}

	/**
	 * Range le palet une fois q'il est attrapé
	 */
	public void rangePalet() {
		// TODO
		// on se dirige tout droit tant que la distance mesurée est < 31 cm

		// si distance mesurée devient < 31 cm, on tourne de 70 degres

		// si on traverse ligne blanche, on pose le palet
	}

	public void dirigerVersPalet() {
		robot.forward();
		// mesurer de manière asynchrone la distance courante
		SampleProvider sampleProvider = capteurs.soundSensor.getDistanceMode();

		// variables pour sauvegarder les mesures
		float[] sample = new float[sampleProvider.sampleSize()];
		float previous = 5; // valeur absurde 5m
		float courant;
		// tant que la distance ne réaugmente pas (cas où on a passé le palet et le mur
		// derrière alors est mesuré), le robot avance
		while (robot.isMoving()) {
			sampleProvider.fetchSample(sample, 0);
			courant = sample[0];
			if (previous < 0.37 && courant > previous)
				robot.stop();
			previous = courant;
			System.out.println(courant);
			Delay.msDelay(500);
		}

		// se recaliber si il faut (si distance mesurée change brusquement hors cas <
		// 31cm)

	}

	/**
	 * Attrape un palet. A utiliser quand le palet n'est plus détecté alors qu'on
	 * avance vers lui, le robot est alors à moins de 31 cm. Renvoie 0 si un palet
	 * est attrapé, renvoie 1 sinon
	 */
	public int attrapePalet() {
		// ouvre pinces
		robot.ouvrePinces();

		// Le robot avance de 30 cm
		robot.travel(3000);

		// on attrape le palet avec un peu de chance
		robot.fermePinces();

		// si le bouton avant est pressé, c'est qu'on a attrapé un palet
		if (capteurs.touchSensor.isPressed())
			return 1;
		else
			return 0;
	}

	/**
	 * Scanne alentour pour chercher des palets. S'oriente alors vers le palet
	 * potentiel le plus proche. Renvoie 0 si un palet a été trouvé, renvoie 1 sinon
	 */
	public int scanPalet() {
		SampleProvider sampleProvider = capteurs.soundSensor.getDistanceMode();

		/*
		 * // projet tourne sur lui meme sur 360 degres et récupère tout du long les //
		 * mesures dans l'arraylist distances int nbMesures = 0; ArrayList<Float>
		 * distances = new ArrayList<Float>(); lib.rotate(360, true); while
		 * (lib.isMoving()) { float[] sample = new float[sampleProvider.sampleSize()];
		 * sampleProvider.fetchSample(sample, 0); distances.add(sample[0]); if
		 * (nbMesures % 1000 == 0) System.out.println(sample[0]); nbMesures++; }
		 * 
		 */

		// Autre version de prises de mesure. On prend une mesure par degre en deleyant
		// chaque mesure
		// en fonction de la vitesse de rotation
		int nbMesures = 0; // le nombre de mesures ne vaudra pas exactement 360
		ArrayList<Float> distances = new ArrayList<Float>(); // on récupère les mesures dans une arraylist
		float[] sample = new float[sampleProvider.sampleSize()];
		robot.rotate(360, true);
		while (robot.isMoving()) {
			sampleProvider.fetchSample(sample, 0); // on fait une mesure
			distances.add(sample[0]); // on l'ajoute à l'array list
			nbMesures++;
			// on délaie la prochaine mesure de : temps total de rotation / nombre de
			// mesures voules
			// <=> (nombre de degres parcourus / vitesse de rotation) en ms / nombre de
			// mesures voules
			Delay.msDelay(1000 * 360 / ((int) robot.getAngularSpeed()) / 360);
		}

		/***
		 * On récupère dans la variable min la plus petite distance qui peut
		 * correspondre à un palet
		 ****/
		float min = (float) 5.0;// initialisation
		int quantiemeMesure = -1; // ordre de la mesure
		float courant;
		for (int i = 0; i < distances.size(); i++) {
			courant = distances.get(i);
			if (courant < min && courant > DISTANCE_MIN_PALET) { // si courant est une distance qui peut correspondre à
																	// un
																	// palet
				min = courant;
				quantiemeMesure = i; // l'index corresond au nombre de mesures / à la combientieme mesure
			}
		}
		/*************************************************************************************************/
		double degreParMesure = 360.0 / nbMesures; // degre parcouru entre deux mesures
		double degrePalet = degreParMesure * quantiemeMesure;
		// TESTS///////

		System.out.println("nbMesures : " + nbMesures);
		// System.out.println("degreParMesure : " + degreParMesure);
		Button.ENTER.waitForPressAndRelease();

		// projet rotate vers la direction qui correpond à la distance min
		System.out.println("min : " + min);
		Button.ENTER.waitForPressAndRelease();
		System.out.println("degreParMesure : " + degreParMesure);
		System.out.println("quantiemeMesure : " + quantiemeMesure);
		System.out.println("degre : " + degrePalet);
		Button.ENTER.waitForPressAndRelease();

		// Si aucune mesure min mesurée (que des infinity ou < 31cm)
		if (quantiemeMesure == -1) {
			return -1;
		}
		// Sinon

		// (degre entre deux mesures) * nbmesures => degre vers distance min
		if (degrePalet < 180)
			robot.rotate(degrePalet + 3); // on rajoute quelque degres pour ne pas viser le bord
											// du palet mais son centre (min correpond bord du palet j'ai l'impression)
		else
			robot.rotate(-360 + degrePalet/* 0 + 2 */);

		// TEST
		sampleProvider.fetchSample(sample, 0);
		System.out.println("distance lue : " + sample[0]);
		Button.ENTER.waitForPressAndRelease();

		return 0;
	}

	public void test() {
		// Button.ENTER.waitForPressAndRelease();

		dirigerVersPalet();

		// attrapePalet();
		// ouvre pinces
		robot.ouvrePinces();

		robot.forward();
		long beginning = System.currentTimeMillis();
		boolean attraped = false;
		// on essaie pendant 7 secondes de capter un palet touché
		while (System.currentTimeMillis() - beginning > 8000 && !attraped) {
			if (capteurs.touchSensor.isPressed()) {
				// on attrape le palet avec un peu de chance
				robot.stop();
				robot.fermePinces();
				attraped = true;
				System.out.println("attraped");
			}
		}
		if (!attraped) {
			// Si au bout de 8s le capteur de toucher n'a rien détecté
			robot.stop();
			robot.fermePinces();
			System.out.println("louped");
		}
		Button.ENTER.waitForPressAndRelease();

		/***
		 * On récupère dans la variable min la plus petite distance qui peut
		 * correspondre à un palet
		 ****/
		/*
		 * float min = (float) 5.0;// initialisation float courant; for (int i = 0 ; i <
		 * distances.size() ; i++) { courant = distances.get(i); if (courant < min) { //
		 * si courant est une distance qui peut correspondre à un // palet min =
		 * courant; } }
		 */
		/*************************************************************************************************/
	}

}
