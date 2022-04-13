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

	final private double DISTANCE_MIN_PALET = 0.321; // distance pour laquelle le palet n'est plus d�tect�

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
		etat = SCAN_PALETS; // au d�but, on va chercher la position des palets
	}

	public void run() {
		// while non fini
		switch (etat) {
		case SCAN_PALETS:
			// on ferme les pinces (il faut v�rifier si les pinces sont d�j� ferm�es)
			// robot.fermePinces();

			// on cherches les palets potentiels
			if (scanPalet() == 0)
				etat = DIRIGER_VERS_PALET;
			// Sinon on continue � scanner pour le moment
			break;

		case DIRIGER_VERS_PALET:
			if (dirigerVersPalet() == 0)
				etat = ATTRAPER_PALET;
			else
				etat = SCAN_PALETS;
			break;

		case ATTRAPER_PALET:
			if (attrapePalet() == 0) // si on attrape bien un palet, on va le ranger
				etat = RANGER_PALET_ATTRAPE;
			else
				// A VOIR : //TODO
				// sinon on verifie tr�s rapidement si jamais un palet a �t� attrap� par les
				// pinces
				// mais n'a pas touch� le balai du capteur de toucher
				//etat = verifAttrapPalet();
				etat = SCAN_PALETS;
			break;

		case RANGER_PALET_ATTRAPE:
			rangePalet();
			// une fois que le palet est pos�, on cherche un nouveau palet
			etat = SCAN_PALETS;
			break;
		}
	}

	/**
	 * Range le palet une fois q'il est attrap�
	 */
	public void rangePalet() {
		// TODO
		// on se dirige tout droit tant que la distance mesur�e est < 31 cm

		// si distance mesur�e devient < 31 cm, on tourne de 70 degres

		// si on traverse ligne blanche, on pose le palet
	}

	/**
	 * Fait bouger le robot en direction du palet potentiel d�tect� auparavant
	 * 
	 * @return 0 si le robot vient de se positionner � 30 cm d'un palet (a priori),
	 *         1 si on �tait sur le point de toucher un mur(on a pas trouv� de
	 *         palet)
	 * 
	 */
	public int dirigerVersPalet() {
		robot.forward();
		// mesurer de mani�re asynchrone la distance courante
		SampleProvider sampleProvider = capteurs.soundSensor.getDistanceMode();

		// variables pour sauvegarder les mesures
		float[] sample = new float[sampleProvider.sampleSize()];
		float previous = 10; // valeur absurde 10m
		float courant;
		// tant que la distance mesur�e ne r�augmente pas (ce serait le cas o� on a
		// pass� le palet et le mur
		// derri�re est alors mesur�), le robot avance
		do {
			// on mesure la distance
			sampleProvider.fetchSample(sample, 0);
			courant = sample[0];
			// on �tait tres proche de quelquechose (un palet peut �tre) et cette chose
			// n'est plus visible
			if (previous < 0.37 && courant > previous) {
				robot.stop();
				// A priori, on est juste devant un palet
				System.out.println("devant un palet");//
				return 0;
			}
			previous = courant;
			System.out.println(courant);
			Delay.msDelay(200);
			// on arr�te la boucle si on va toucher un mur
		} while (courant > 0.12);
		robot.stop();
		// on a loup� le palet

		System.out.println("c'etait un mur");
		return 1;

		// se recaliber si il faut (si distance mesur�e change brusquement hors cas <
		// 31cm)

	}

	/**
	 * Attrape un palet. A utiliser quand le palet n'est plus d�tect� alors qu'on
	 * avance vers lui, le robot est alors � moins de 31 cm. Renvoie 0 si un palet
	 * est attrap�, renvoie 1 sinon
	 */
	public int attrapePalet() {
		System.out.println("attrapePalet()");
		// on ouvre les pinces pour attraper le palet
		robot.ouvrePinces();

		// on avance vers le palet
		robot.forward();

		// on mesure de mani�re asynchrone la distance de l'objet devant le robot
		SampleProvider sampleProvider = capteurs.soundSensor.getDistanceMode();
		float[] sample = new float[sampleProvider.sampleSize()];
		float distanceMesuree = 99; // valeur absurde
		// on arr�te d'essayer d'attraper au bout de 8 secondes ou si on s'apprete �
		// toucher un mur
		long beginning = System.currentTimeMillis();
		while (System.currentTimeMillis() - beginning < 8000 && distanceMesuree > 0.12) {
			// le senseur a senti quelque chose (un palet normalement)
			if (capteurs.touchSensor.isPressed()) {
				robot.stop();
				robot.fermePinces();
				System.out.println("attraped"); // TEST
				return 0;
			}
			sampleProvider.fetchSample(sample, 0);
			distanceMesuree = sample[0];
		}

		// On abandonne au bout de 8 secondes ou si le robot allait toucher un mur
		robot.stop();
		robot.fermePinces();
		System.out.println("louped"); // TEST
		return 1;
	}

	/**
	 * Verifie tr�s rapidement si jamais un palet a �t� attrap� par les pinces mais
	 * n'a pas touch� le balai du capteur de toucher. R�ouvre les pinces et avance
	 * d'un chouia. Si un palet �tait attrap�, le d�tecteur de toucher le d�tecte
	 * maintenant.
	 * 
	 * @return RANGER_PALET_ATTRAPE si il y avait un palet, SCAN_PALETS sinon
	 */
	public int verifAttrapPalet() {
		System.out.println("verifAttrPalet()"); // TEST
		robot.ouvrePinces();
		robot.travel(200, true); // on bouge de 2cm

		// on ouvre les pinces et on avance une seconde pour voir si un palet est
		// d�tect�
		long beginning = System.currentTimeMillis();
		while (System.currentTimeMillis() - beginning < 500) {

			// il y avait bien un palet attrap�
			if (capteurs.touchSensor.isPressed()) {
				robot.stop();
				robot.fermePinces();
				System.out.println("un palet finalement"); // TEST
				// on renvoie l'etat suivant
				return RANGER_PALET_ATTRAPE;
			}
		}
		// Pas de palet d�tect� :
		robot.stop();
		robot.fermePinces();
		System.out.println("pas de palet"); // TEST
		// Il faut en chercher un palet
		return SCAN_PALETS;
	}

	/**
	 * Scanne alentour pour chercher des palets. S'oriente alors vers le palet
	 * potentiel le plus proche. Renvoie 0 si un palet a �t� trouv�, renvoie 1 sinon
	 */
	public int scanPalet() {
		SampleProvider sampleProvider = capteurs.soundSensor.getDistanceMode();

		/*
		 * // projet tourne sur lui meme sur 360 degres et r�cup�re tout du long les //
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
		ArrayList<Float> distances = new ArrayList<Float>(); // on r�cup�re les mesures dans une arraylist
		float[] sample = new float[sampleProvider.sampleSize()];
		robot.rotate(360, true);
		while (robot.isMoving()) {
			sampleProvider.fetchSample(sample, 0); // on fait une mesure
			distances.add(sample[0]); // on l'ajoute � l'array list
			nbMesures++;
			// on d�laie la prochaine mesure de : temps total de rotation / nombre de
			// mesures voules
			// <=> (nombre de degres parcourus / vitesse de rotation) en ms / nombre de
			// mesures voules
			Delay.msDelay(1000 * 360 / ((int) robot.getAngularSpeed()) / 360);
		}

		/***
		 * On r�cup�re dans la variable min la plus petite distance qui peut
		 * correspondre � un palet
		 ****/
		float min = (float) 5.0;// initialisation
		int quantiemeMesure = -1; // ordre de la mesure
		float courant;
		for (int i = 0; i < distances.size(); i++) {
			courant = distances.get(i);
			if (courant < min && courant > DISTANCE_MIN_PALET) { // si courant est une distance qui peut correspondre �
																	// un
																	// palet
				min = courant;
				quantiemeMesure = i; // l'index corresond au nombre de mesures / � la combientieme mesure
			}
		}
		/*************************************************************************************************/
		double degreParMesure = 360.0 / nbMesures; // degre parcouru entre deux mesures
		double degrePalet = degreParMesure * quantiemeMesure;
		// TESTS///////

		System.out.println("nbMesures : " + nbMesures);
		// System.out.println("degreParMesure : " + degreParMesure);
		Button.ENTER.waitForPressAndRelease();

		// projet rotate vers la direction qui correpond � la distance min
		System.out.println("min : " + min);
		Button.ENTER.waitForPressAndRelease();
		System.out.println("degreParMesure : " + degreParMesure);
		System.out.println("quantiemeMesure : " + quantiemeMesure);
		System.out.println("degre : " + degrePalet);
		Button.ENTER.waitForPressAndRelease();

		// Si aucune mesure min mesur�e (que des infinity ou < 31cm)
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

		/*
		 * // TEST sampleProvider.fetchSample(sample, 0);
		 * System.out.println("distance lue : " + sample[0]);
		 * Button.ENTER.waitForPressAndRelease();
		 */

		return 0;
	}

	public void test() {
		// Button.ENTER.waitForPressAndRelease();
		// robot.fermePinces();
		// robot.ouvrePinces();

		// dirigerVersPalet();
		// attrapePalet();
		verifAttrapPalet();
		Button.ENTER.waitForPressAndRelease();

		/***
		 * On r�cup�re dans la variable min la plus petite distance qui peut
		 * correspondre � un palet
		 ****/
		/*
		 * float min = (float) 5.0;// initialisation float courant; for (int i = 0 ; i <
		 * distances.size() ; i++) { courant = distances.get(i); if (courant < min) { //
		 * si courant est une distance qui peut correspondre � un // palet min =
		 * courant; } }
		 */
		/*************************************************************************************************/
	}

}
