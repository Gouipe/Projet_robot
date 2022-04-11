package robot;

import lejos.hardware.Button;

public class Test {

	public static void main(String[] args) {
		System.out.println("coucou");
		Button.ENTER.waitForPressAndRelease();
	}
}