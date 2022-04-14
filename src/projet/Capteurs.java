package projet;

import java.util.HashMap;
import java.util.Map;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;
import lejos.utility.Delay;
import lib.TestColor;

public class Capteurs {
	
	// les attributs
	EV3UltrasonicSensor soundSensor;
	TouchSensor touchSensor;
	ColorSensor colorSensor;
	//Map<String,Boolean> couleursDetectees; pour stoker les valeurs des couleurs
	
	//les contructeurs
	public Capteurs(EV3UltrasonicSensor soundSensor, TouchSensor touchSensor, ColorSensor senseurDeCouleur) {
		this.soundSensor = soundSensor;
		this.touchSensor = touchSensor;
		this.colorSensor = senseurDeCouleur;
		//couleursDetectees = new HashMap<>();
	}
	public Capteurs(EV3UltrasonicSensor soundSensor, TouchSensor touchSensor) {
		this.soundSensor = soundSensor;
		this.touchSensor = touchSensor;
	}
	
	
	
	// les methodes
	public boolean aTraverserLigneBlanche(){
		int couleurID = this.colorSensor.colorSensor.getColorID();
		while(couleurID != 6 ){
			couleurID = this.colorSensor.colorSensor.getColorID();	
			//System.out.println("couleurID");
			//System.out.println(couleurID);
		}	
		return false;
		
		
	}
	
	public boolean ligneBlancheDetectee() {
		return this.colorSensor.colorSensor.getColorID() == 6;	
	}


	
}