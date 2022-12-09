package RescueLine;

import lejos.hardware.BrickFinder;
import lejos.hardware.Keys;
import lejos.hardware.Sound;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;
import lejos.utility.Delay;

public class AvoidBlock  {
	public static void main(String[] args) throws Exception {
		
		EV3UltrasonicSensor sonicSensor = new EV3UltrasonicSensor(SensorPort.S2);
		SampleProvider sonicdistance = sonicSensor.getDistanceMode();
		SampleProvider average = new MeanFilter(sonicdistance, 5);
		float [] ultraSample = new float[average.sampleSize()];

		sonicSensor.fetchSample(ultraSample, 0);
		float x = ultraSample[0] *100;
		int distance = (int)x;
		boolean isBlock;
		if(20 < distance && distance < 50) {
			isBlock = true;
		}
		else
			isBlock = false;
		
		while(isBlock) {
			if(20<distance && distance <50) {
				pilot.forward();
			}
			else if(distance<20) {
				pilot.stop();
				pilot.rotate(30);
			}
			else if(distance >50 && distance < 60){
				pilot.rotate(-30);
			}
			else if(LeftColorSensor.getColorID() == Color.BLACK && RightColorSensor.getColorID() == Color.BLACK) {
				pilot.rotate(90);
				isBlock = false;
				break;
			}
			else
				pilot.rotate(-90);
		}
	}
}

