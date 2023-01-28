package frc.robot;


import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

import java.nio.Buffer;
import java.rmi.server.Operation;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
//import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotState;

//import edu.wpi.first.hal.PDPJNI;
import edu.wpi.first.wpilibj.Compressor;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;


public class Robot extends TimedRobot {

	// Sensors

	public Gyro driveTrainGryo = new AnalogGyro(0);
	public CANSparkMax testMotor = new CANSparkMax(5, MotorType.kBrushless);
	public RelativeEncoder encoderTestMotor;

	// Drivetrain
	public DriveTrain driveTrain = new DriveTrain(0, 1, 2, 3);


	// Joysticks
	public Joystick flightStickLeft = new Joystick(0);
	public Joystick flightStickRight = new Joystick(1);


	public void robotInit() {
	}

	public void disabledInit() {
		driveTrain.SetBreak();
	}

	public void autonomousInit() {
	}

	public void autonomousPeriodic() {
	}

	public void teleopInit() {
		flightStickLeft = new Joystick(1);
		flightStickRight = new Joystick(0);

	}

	public void teleopPeriodic() {
		testMotor.set(0.0f);
		driveTrain.SetCoast();
		ControllerDrive();

		driveTrain.Update();
	}

	public void testInit() {
		driveTrainGryo.reset();
	}

	public void testPeriodic() {
		encoderTestMotor = testMotor.getEncoder();
		testMotor.setIdleMode(IdleMode.kCoast);
		System.out.println(encoderTestMotor.getVelocity());
		testMotor.set(0);






		// System.out.println(driveTrainGryo.getAngle() + " gryo");
		// if(driveTrainGryo.getAngle() <=  90){
		// 	driveTrain.SetLeftSpeed(0.3f);
		// 	driveTrain.SetRightSpeed(-0.3f);
		// // 	System.out.println("TRying but failing");
		// }else{
		// 	driveTrain.SetBothSpeed(0);
		// }
		// driveTrain.Update();
	}

	public void CompressorsOn() {
	}

	public void CompressorOff() {
	}

	public float TranslateController(float input) {
		float deadzone = 0.15f;
		if (input > -deadzone && input < deadzone) {
			input = 0.0f;
		}
		float a = 0.7f;
		float output = (a * input * input * input) + (1 - a) * input;
		return output;
	}

	public void ControllerDrive() {
		float horJoystick = TranslateController((float) flightStickLeft.getRawAxis(0));
		float verJoystick = TranslateController((float) flightStickRight.getRawAxis(1));

		driveTrain.SetRightSpeed(-verJoystick + -horJoystick);
		System.out.println(-verJoystick + -horJoystick);
		driveTrain.SetLeftSpeed(-verJoystick + horJoystick);
		driveTrain.SetCoast();
	}

	public void DrivetrainBrakes(boolean brakes) {
		if (brakes = true) {
			driveTrain.SetBreak();
		} else {
			driveTrain.SetCoast();
		}
	}
}