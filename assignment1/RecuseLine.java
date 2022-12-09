import com.robotic.ColorSensor;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Keys;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.LCD;
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

	public class RecuseLine
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
	        LEFT_MOTOR.setSpeed(110);
	        RIGHT_MOTOR.setSpeed(110);

	        while(!Button.ESCAPE.isDown()) {


	        		if(LeftColorSensor.getColorID() == Color.BLACK) {
	        			Delay.msDelay(200);
	        			pilot.stop();
	        			pilot.rotate(15);
		        		LEFT_MOTOR.setSpeed(110);
		    	        RIGHT_MOTOR.setSpeed(110);
	        		}else if(RightColorSensor.getColorID() == Color.BLACK) {
	        			Delay.msDelay(200);
	        			pilot.stop();
		        		LEFT_MOTOR.setSpeed(110);
		    	        RIGHT_MOTOR.setSpeed(110);

	        		}
	    	     else if(LeftColorSensor.getColorID()==Color.GREEN) {
	        		Delay.msDelay(2000);
	        		pilot.stop();
	        		pilot.setAngularSpeed(15);
	        		pilot.rotate(90);
	        		pilot.setLinearSpeed(10);
	        		pilot.travel(40);
	        		LEFT_MOTOR.setSpeed(110);
	    	        RIGHT_MOTOR.setSpeed(110);


	        	}
	        	else if(RightColorSensor.getColorID()==Color.GREEN) {
	        		Delay.msDelay(2000);
	        		pilot.stop();
	        		pilot.setAngularSpeed(15);
	        		pilot.rotate(-90);
	        		pilot.setLinearSpeed(10);
	        		pilot.travel(40);
	        		LEFT_MOTOR.setSpeed(110);
	    	        RIGHT_MOTOR.setSpeed(110);
		}
	        	else if(LeftColorSensor.getColorID()==Color.GREEN) {
	        		if(RightColorSensor.getColorID()==Color.GREEN) {
	        			LCD.drawString("Detects two GREEN", 0, 0);
	        			pilot.stop();
	        			Delay.msDelay(300);
		        		pilot.setAngularSpeed(15);
	        			pilot.rotate(180);
		        		LEFT_MOTOR.setSpeed(110);
		    	        RIGHT_MOTOR.setSpeed(110);
	        		}
	        	}
	        	else if(RightColorSensor.getColorID()==Color.GREEN) {
	        		if(LeftColorSensor.getColorID()==Color.GREEN) {
	        			LCD.drawString("Detects two GREEN", 0, 0);
	        			pilot.stop();
	        			Delay.msDelay(300);
		        		pilot.setAngularSpeed(15);
	        			pilot.rotate(180);
		        		LEFT_MOTOR.setSpeed(110);
		    	        RIGHT_MOTOR.setSpeed(110);
	        		}
	        	}
	        	else if (LeftColorSensor.getColorID()==Color.RED||RightColorSensor.getColorID()==Color.RED)
				{
					pilot.stop();
					Delay.msDelay(500);
					LEFT_MOTOR.forward();
					RIGHT_MOTOR.forward();
					LEFT_MOTOR.setSpeed(110);
	    	        RIGHT_MOTOR.setSpeed(110);

				}
	        	else if(LeftColorSensor.getColorID()==Color.BLACK||RightColorSensor.getColorID()==Color.BLACK) {
	        		if(LeftColorSensor.getColorID()==Color.GREEN||RightColorSensor.getColorID()==Color.GREEN) {
	        			LEFT_MOTOR.forward();
						RIGHT_MOTOR.forward();
						LEFT_MOTOR.setSpeed(110);
		    	        RIGHT_MOTOR.setSpeed(110);
	        		}
	        	}
	        	else
	    	        LEFT_MOTOR.forward();
	    	        RIGHT_MOTOR.forward();
	        }

	      //part3

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
		}
	}
}
