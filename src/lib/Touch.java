package lib;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.utility.Delay;

public class Touch
{
    public static void main(String[] args)
    {
        try {
        	log("Program Starting");

        projet.TouchSensor uTouch = new projet.TouchSensor(SensorPort.S2);
        waitForTouch(uTouch);

        log("Program Ending");
        
        } catch (Throwable t) {
			t.printStackTrace();
			Delay.msDelay(10000);
			System.exit(0);
		}
    }
   

    private static void waitForTouch(projet.TouchSensor uTouch)
    {
        log("Waiting for press on Touch Sensor");

        while (! uTouch.isPressed())
        {
            Delay.msDelay(100);
        }

        log("Touch Sensor pressed.");
    }


    private static void log(String msg)
    {
        System.out.println("log>\t" + msg);
    }
}

