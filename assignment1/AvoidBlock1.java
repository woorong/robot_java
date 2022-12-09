package com.robotic.linefollower;




import com.robotic.ColorSensor;
import lejos.hardware.Button;
import lejos.hardware.BrickFinder;
import lejos.hardware.Keys;
import lejos.hardware.ev3.EV3;
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

public class AvoidBlock1  {
	public static void main(String[] args) throws Exception {
		//movement
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
		//ultra
		EV3UltrasonicSensor UltraSensor = new EV3UltrasonicSensor(SensorPort.S4);
		SampleProvider sonicdistance2 = UltraSensor.getDistanceMode();
		SampleProvider average2 = new MeanFilter(sonicdistance2, 5);
		float [] sample2 = new float[average2.sampleSize()];
		UltraSensor.fetchSample(sample2, 0);
		float y = sample2[0] *100;
		int distance2 = (int)y;

		//irSonic
		EV3IRSensor irSensor = new EV3IRSensor(SensorPort.S3);
		SampleProvider sonicdistance1 = irSensor.getDistanceMode();
		SampleProvider average1 = new MeanFilter(sonicdistance1, 5);
		float [] sample1 = new float[average1.sampleSize()];
		irSensor.fetchSample(sample1, 0);
		float x = sample1[0] *100;
		int distance1 = (int)x;


		//speed
        LEFT_MOTOR.setSpeed(120);
        RIGHT_MOTOR.setSpeed(120);
        LEFT_MOTOR.forward();
        RIGHT_MOTOR.forward();

        //decision
		while(!Button.ESCAPE.isDown()) {
			int count = 0;
			if(LeftColorSensor.getColorID() == Color.BLACK) {
	        		pilot.stop();
	        		pilot.rotate(15);
	                LEFT_MOTOR.setSpeed(120);
	                RIGHT_MOTOR.setSpeed(120);
	        	}
				else if(RightColorSensor.getColorID() == Color.BLACK){
					pilot.stop();
					pilot.rotate(-15);
					LEFT_MOTOR.setSpeed(120);
					RIGHT_MOTOR.setSpeed(120);
				}else if(LeftColorSensor.getColorID() == Color.BLACK && RightColorSensor.getColorID() == Color.BLACK) {
					pilot.travel(40);
					pilot.rotate(-90);
					LEFT_MOTOR.setSpeed(120);
					RIGHT_MOTOR.setSpeed(120);
			}

			//obstacles
			while(true) {
				LEFT_MOTOR.forward();
				RIGHT_MOTOR.forward();
				LEFT_MOTOR.setSpeed(120);
				RIGHT_MOTOR.setSpeed(120);
				if(distance2 < 0.2) {
					pilot.stop();
					pilot.travel(-40);
					pilot.rotate(-90);
				}
				if(distance1 < 0.1) {
					pilot.stop();
					pilot.rotate(-10);
				}
				if(distance1 > 0.3) {
					pilot.rotate(90);
				}
			}

		}
	}
}
