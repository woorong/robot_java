	package RescueLine;

	import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Keys;
	import lejos.hardware.Sound;
	import lejos.hardware.ev3.EV3;
	import lejos.hardware.lcd.TextLCD;
	import lejos.hardware.motor.EV3LargeRegulatedMotor;
	import lejos.hardware.port.MotorPort;
	import lejos.hardware.port.Port;
	import lejos.hardware.port.SensorPort;
	import lejos.hardware.sensor.EV3ColorSensor;
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

	public class part123  {
		public static void main(String[] args) throws Exception {
			EV3LargeRegulatedMotor LEFT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.A);
			EV3LargeRegulatedMotor RIGHT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.B);
			
			Wheel LEFT_WHEEL = WheeledChassis.modelWheel(LEFT_MOTOR, 5.5).offset(-6.6);
			Wheel RIGHT_WHEEL = WheeledChassis.modelWheel(RIGHT_MOTOR, 5.5).offset(6.6);
			
			Chassis chassis = new WheeledChassis(new Wheel[] {LEFT_WHEEL, RIGHT_WHEEL}, WheeledChassis.TYPE_DIFFERENTIAL);
			
			MovePilot pilot = new MovePilot(chassis);
			EV3ColorSensor LeftColorSensor = new EV3ColorSensor(SensorPort.S2);
			EV3ColorSensor RightColorSensor = new EV3ColorSensor(SensorPort.S1);
			EV3 ev3brick = (EV3) BrickFinder.getLocal();
			EV3UltrasonicSensor sonicSensor = new EV3UltrasonicSensor(SensorPort.S3);
			
			SampleProvider sonicdistance = sonicSensor.getDistanceMode();
			
			SampleProvider average = new MeanFilter(sonicdistance, 5);
			
			float [] ultraSample = new float[average.sampleSize()];
			
			
			//part1
			pilot.setLinearSpeed(15);
			LEFT_MOTOR.setSpeed(15);
			RIGHT_MOTOR.setSpeed(15);
			
			pilot.forward();
			boolean isGo = false;
			while(isGo) {
				while(LeftColorSensor.getColorID() == Color.BLACK && RightColorSensor.getColorID() == Color.WHITE) {
					pilot.stop();
					pilot.rotate(-10);
					Delay.msDelay(100);
					pilot.forward();
				}
				while(LeftColorSensor.getColorID() == Color.WHITE && LeftColorSensor.getColorID() == Color.BLACK) {
					pilot.stop();
					pilot.rotate(10);
					Delay.msDelay(100);
					pilot.forward();
				}
				while(LeftColorSensor.getColorID() == Color.BLACK && LeftColorSensor.getColorID() == Color.BLACK) {
					pilot.stop();
					isGo = false;
					//bbCase(lDetect, rDetect);
				}
				while(LeftColorSensor.getColorID() == Color.RED && LeftColorSensor.getColorID() == Color.RED)
					isGo = false;

				}
			Delay.msDelay(50);
			
			//part2
			LeftColorSensor.setFloodlight(true);
	        RightColorSensor.setFloodlight(true);
	        LeftColorSensor.getRGBMode();
	        RightColorSensor.getRGBMode();
	        
	        LeftColorSensor.getColorIDMode();
	        RightColorSensor.getColorIDMode();
	       
	        
			pilot.forward();
			pilot.setLinearSpeed(15);
			
			while (true) {
				
				if(LeftColorSensor.getColorID()==Color.GREEN&&RightColorSensor.getColorID()==Color.WHITE) {
					pilot.stop();
					Delay.msDelay(500);
					pilot.rotate(90);
				}
				else if(RightColorSensor.getColorID()==Color.GREEN&&LeftColorSensor.getColorID()==Color.WHITE) {
					pilot.stop();
					Delay.msDelay(500);
					pilot.rotate(-90);
				}
				else if(LeftColorSensor.getColorID()==Color.GREEN&&RightColorSensor.getColorID()==Color.GREEN) {
					pilot.stop();
					Delay.msDelay(500);
					pilot.rotate(180);
				}
				else if(LeftColorSensor.getColorID()==Color.RED||RightColorSensor.getColorID()==Color.RED) {
					pilot.stop();
					Delay.msDelay(500);
				}
				
				else {
					pilot.forward();
				}
			
				if(Button.ESCAPE.isDown()) {
					pilot.stop();
					System.exit(0);
				}
				Delay.msDelay(20);
			}
			
			
			
			//part3
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
