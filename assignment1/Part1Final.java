


import java.awt.Robot;

import javax.net.ssl.SNIHostName;

import RescueLine.ColorSensor;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Keys;
import lejos.hardware.ev3.EV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.Color;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.ArcRotateMoveController;
import lejos.robotics.navigation.MovePilot;
	import lejos.utility.Delay;

	public class Part1Final
	{


		public static void main(String[] args) {
			//
			EV3LargeRegulatedMotor LEFT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.A);
			EV3LargeRegulatedMotor RIGHT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.B);

			EV3 ev3brick = (EV3) BrickFinder.getLocal();

			Keys buttons = ev3brick.getKeys();


			Wheel wheel1 = WheeledChassis.modelWheel(LEFT_MOTOR, 56).offset(65);
			Wheel wheel2 = WheeledChassis.modelWheel(RIGHT_MOTOR, 56).offset(-65);

			Chassis chassis = new WheeledChassis(new Wheel [] {wheel1,  wheel2},
					WheeledChassis.TYPE_DIFFERENTIAL);

			MovePilot pilot = new MovePilot(chassis);

			ColorSensor LeftColorSensor = new ColorSensor(SensorPort.S2);
			ColorSensor RightColorSensor = new ColorSensor(SensorPort.S1);

	        LeftColorSensor.setFloodLight(true);
	        RightColorSensor.setFloodLight(true);

	        LeftColorSensor.setColorIdMode();
	        RightColorSensor.setColorIdMode();

			System.out.println("Ready!");
			buttons.waitForAnyPress();


	        LEFT_MOTOR.forward();
	        RIGHT_MOTOR.forward();
	        LEFT_MOTOR.setSpeed(80);
	        RIGHT_MOTOR.setSpeed(80);

	        while(!Button.ESCAPE.isDown()) {



	        	if(LeftColorSensor.getColorID() == Color.BLACK) {
	        		pilot.stop();
	        		pilot.rotate(10);
	        		LEFT_MOTOR.setSpeed(80);
	    	        RIGHT_MOTOR.setSpeed(80);
	    	        }

	        	else if(RightColorSensor.getColorID() == Color.BLACK) {
	        		pilot.stop();
	        		pilot.rotate(-10);
	        		LEFT_MOTOR.setSpeed(80);
	    	        RIGHT_MOTOR.setSpeed(80);

	        	}
	        	else if(RightColorSensor.getColorID()==Color.BLACK) {
	        		if (LeftColorSensor.getColorID()==Color.BLACK) {
	        		Delay.msDelay(300);
	        		pilot.travel(90);
	        		LEFT_MOTOR.setSpeed(80);
	    	        RIGHT_MOTOR.setSpeed(80);
	        		}
	        	}
	        	else if(LeftColorSensor.getColorID()==Color.BLACK) {
	        		if (RightColorSensor.getColorID()==Color.BLACK) {
	        			Delay.msDelay(300);
		        		pilot.travel(90);
		        		LEFT_MOTOR.setSpeed(80);
		    	        RIGHT_MOTOR.setSpeed(80);
	        		}
	        	}
	        	else {
	                LEFT_MOTOR.forward();
	        		RIGHT_MOTOR.forward();
	        	}
	        }
		}
	}
