package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Bot2 extends TimedRobot {
	// Let's define our constants here. Why? Because they're easy to find when
	// tuning.
	// They are implemented below
	private static final double Kp = 0.5; // <- when i say change Kp, it’s this!
	private static final double Ki = 0.0;
	private static final double Kf = 0.0; // no feed-forward on position control
	private static final double Kd = 0;
	private static final int IZone = 100; // IZone, this is explained below

	Joystick joystick = new Joystick(0);
	TalonSRX _motor = new TalonSRX(1);
	Faults _faults = new Faults(); /* temp to fill with latest faults */

	@Override
	public void robotInit() {

	}

	@Override
	public void autonomousInit() {
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		_motor.configFactoryDefault();

		/**
		 * Quad is the default sensor type
		 */
		_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

		/*************************************************************************
		 * Ramps. The argument is how many seconds it takes to go from 0 to 100% during
		 * voltage control. There is a ramp for both open loop and closed (controlled)
		 * loop
		 */
		_motor.configOpenloopRamp(0.2);
		_motor.configClosedloopRamp(0.2);

		/**
		 * Setting feedbackNotContinuous to true is recommended, to prevent intermittent
		 * connections from causing sensor "jumps" of 4096 (or 1024 for analog) units
		 */
		_motor.configFeedbackNotContinuous(true, 0);

		// Talon setup. Setting up + and - output for max and nominal.
		// These should never really change
		_motor.configNominalOutputForward(0, 0);
		_motor.configNominalOutputReverse(0, 0);
		_motor.configPeakOutputForward(0.07, 0);
		_motor.configPeakOutputReverse(-0.07, 0);

		// set this if your encoder is hooked up backwards (A<--->B)
		// mine is backwards, so I've set it to true
		_motor.setSensorPhase(false);

		/************************
		 * Do not use setInverted to correct sensor orientation with respect to motor
		 * output. setInverted synchronously inverts both signals, ensuring that sensor
		 * phase is maintained. This is a feature that allows you to choose what
		 * direction is considered positive without breaking closed-looping features.
		 */
		_motor.setInverted(true);

		// implementation of tuning constants. First argument is PID slot. We're only
		// using slot 0. If we had multiple tuning constants for the same controller
		// for different purposes, we can also have up to 3 slots pre-programmed.
		// The last number is a timeout- we can set an error vector if for some reason
		// we can’t program our talon and it times out. I’m not doing this here.
		_motor.config_kF(0, Kf, 0); // No feed-forward on position control
		_motor.config_kP(0, Kp, 0); // P factor!
		_motor.config_kI(0, Ki, 0);
		_motor.config_kD(0, Kd, 0); // note- these are defined above so they're easy to find and change.

		/**
		* Grab the 360 degree position of the MagEncoder's absolute
		* position, and intitally set the relative sensor to match.
		*/
		int absolutePosition = _motor.getSensorCollection().getPulseWidthPosition();

		/* Mask out overflows, keep bottom 12 bits */
		absolutePosition &= 0xFFF;
		// if (Constants.kSensorPhase) { // true
		// absolutePosition *= -1;
		// }

		// if (Constants.kMotorInvert) { // true
		absolutePosition *= -1;
		// }

		/* Set the quadrature (relative) sensor to match absolute */
		_motor.setSelectedSensorPosition(0, 0, 30);
	}

	@Override
	public void teleopPeriodic() {
		commandLoop();
		// commandLoopTest();
	}

	private void commandLoopTest(){
		String btn = "n/a";
		if(joystick.getRawButton(1)) btn = "1-" + joystick.getRawButtonPressed(1);
		if(joystick.getRawButton(2)) btn = "2-" + joystick.getRawButtonPressed(2);
		if(joystick.getRawButton(3)) btn = "3-" + joystick.getRawButtonPressed(3);
		if(joystick.getTrigger()) btn = "T-" + joystick.getTriggerPressed(); 

		SmartDashboard.putString("button test:" , btn);
	}

	private void commandLoop() {

		/**************************************************************************
		 * Here's what I'm doing during teleop: If you're NOT holding the trigger, I
		 * operate like a normal motor- just piping Y axis of my joystick over to the
		 * motor. Full stick forward= full motor power. I'm also reading the Z axis on
		 * the joystick. This gives me a number from -1 to 1. When you hit the trigger,
		 * I set the target of the Talon to Z*4096*10 which sets a position target of
		 * anywhere from -40 to 40 full Turns of the encoder wheel. I have the PID set
		 * to get me there as fast as possible, and the talon takes care of it. You'll
		 * also notice there are some SmartDashboard numbers thrown up there to check
		 * accuracy.
		 */


		int _loops = 0;
		double axisY = joystick.getY(); // speed

		SmartDashboard.putNumber("Current Pos:", _motor.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("** B3 Z value:", joystick.getZ());

		double targetPosition = 0.5 * 4096 * 10; //

		// if (joystick.getTrigger())
		if (joystick.getRawButton(3)) {
			_motor.set(ControlMode.Position, targetPosition);
			// if (++_loops >= 10) {
			// _loops = 0;
			// log("button 3 pressed: " + joystick.getRawButtonPressed(3));
			// }
			SmartDashboard.putNumber("Target:", targetPosition);
			
			// SmartDashboard.putBoolean("B3 Pressed:", joystick.getRawButton(3));
		}
		// reguluar teleop. Just give the motor power wherever you have the joystick set
		else if (joystick.getTrigger()) {

			double speed = axisY * 0.5;
			if (Math.abs(speed) < 0.02)
				speed = 0;

			_motor.set(ControlMode.PercentOutput, speed);

			// if (++_loops >= 10) {
			// _loops = 0;
			SmartDashboard.putNumber("B1 axisY", axisY);
			// 
			// SmartDashboard.putBoolean("B3 Pressed:", joystick.getRawButton(3));
			// }
		} else if (joystick.getRawButton(2)) {
			_motor.set(ControlMode.PercentOutput, 0);
		}

		/* check our live faults */
		_motor.getFaults(_faults);

		/* hold down btn1 to print stick values */
		// if (joystick.getRawButton(1)) {
		// SmartDashboard.putNumber("Sensor Vel:" , _motor.getSelectedSensorVelocity());
		// SmartDashboard.putNumber("Sensor Pos:" , _motor.getSelectedSensorPosition());
		// SmartDashboard.putNumber("Out %" , _motor.getMotorOutputPercent());
		SmartDashboard.putBoolean("Out Of Phase:", _faults.SensorOutOfPhase);
		// }

	}

	@Override
	public void testInit() {
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}

	public void log(String s) {
		System.out.println(s);
	}

}
