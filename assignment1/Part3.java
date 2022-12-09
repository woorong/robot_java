



import com.robotic.ColorSensor;
import lejos.hardware.Button;
import lejos.hardware.BrickFinder;
import lejos.hardware.Keys;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.filter.MeanFilter;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;

public class AvoidBlock  {
	public static void main(String[] args) throws Exception {
		//action
		EV3LargeRegulatedMotor LEFT_MOTOR = new EV3LargeRegulatedMotor(
				MotorPort.A);
		EV3LargeRegulatedMotor RIGHT_MOTOR = new EV3LargeRegulatedMotor(
				MotorPort.B);
		Wheel wheel1 = WheeledChassis.modelWheel(LEFT_MOTOR, 56).offset(65);
		Wheel wheel2 = WheeledChassis.modelWheel(RIGHT_MOTOR, 56).offset(-65);

		Chassis chassis = new WheeledChassis(new Wheel [] {wheel1,  wheel2},
				WheeledChassis.TYPE_DIFFERENTIAL);
		MovePilot pilot = new MovePilot(chassis);

		//color
		ColorSensor LeftColorSensor = new ColorSensor(SensorPort.S2);
		ColorSensor RightColorSensor = new ColorSensor(SensorPort.S1);

		//button
		EV3 ev3brick =(EV3) BrickFinder.getLocal();
		Keys buttons = ev3brick.getKeys();
		//start
		System.out.println("Ready!");
		buttons.waitForAnyPress();

		TextLCD lcddisplay = ev3brick.getTextLCD();

		//ultra
		EV3UltrasonicSensor UltraSensor = new EV3UltrasonicSensor(SensorPort.S4);
		SampleProvider sonicdistance2 = UltraSensor.getDistanceMode();
		SampleProvider average2 = new MeanFilter(sonicdistance2, 5);
		float [] sample2 = new float[average2.sampleSize()];

		//irSonic
		EV3IRSensor irSensor = new EV3IRSensor(SensorPort.S3);
		SampleProvider sonicdistance1 = irSensor.getDistanceMode();
		SampleProvider average1 = new MeanFilter(sonicdistance1, 5);
		float [] sample1 = new float[average1.sampleSize()];






        while(!Button.ESCAPE.isDown()) {
    		//start
            LEFT_MOTOR.setSpeed(80);
            RIGHT_MOTOR.setSpeed(80);
            LEFT_MOTOR.forward();
            RIGHT_MOTOR.forward();

    		UltraSensor.fetchSample(sample2, 0);
    		irSensor.fetchSample(sample1, 0);

    		float y = sample2[0] *100;
    		float x = sample1[0];

    		boolean BlockIr = false;
    		boolean BlockUltra = false;

    		if(y < 8) {
    			BlockUltra = true;
    		}

    		if(y > 30) {
    			BlockUltra = false;
    		}

    		if(x < 10) {
    			BlockIr = true;
    		}

    		if(x > 20) {
    			BlockIr = false;

    		}


    		while(BlockUltra == true && BlockIr == false) {
    			pilot.stop();
    			Delay.msDelay(1500);
    			pilot.setAngularSpeed(10);
    			pilot.rotate(-90);
    		}

    		while(BlockUltra == false && BlockIr == true) {
    			RIGHT_MOTOR.forward();
    			LEFT_MOTOR.forward();
    			Delay.msDelay(200);
    		}
    		while(BlockUltra == false && BlockIr == false) {
    			pilot.stop();
    			pilot.setLinearSpeed(10);
    			pilot.travel(40);
    			Delay.msDelay(500);
    			pilot.rotate(90);
    		}

    		while(RightColorSensor.getColorID() == Color.BLACK && LeftColorSensor.getColorID() == Color.BLACK) {
    			Delay.msDelay(1500);
    			pilot.stop();
    			pilot.setAngularSpeed(10);
    			pilot.rotate(-90);
    		}

    		while(true) {
    			if(RightColorSensor.getColorID() == Color.BLACK) {
    				Delay.msDelay(500);
    				pilot.stop();
    				pilot.rotate(-15);
    			}else if(LeftColorSensor.getColorID() == Color.BLACK) {
    				Delay.msDelay(500);
    				pilot.stop();
    				pilot.rotate(15);
    			}
    		}
        }
	}
}
