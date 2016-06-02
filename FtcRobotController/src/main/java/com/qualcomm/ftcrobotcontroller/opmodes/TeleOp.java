
/**
 * Created by maddi_000 on 2/1/2016.
 */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import java.lang.Override;
import java.lang.String;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class TeleOp extends OpMode {

// Declare the hardware

	DcMotor motorRight1;
	DcMotor motorRight2;
	DcMotor motorLeft1;
	DcMotor motorLeft2;
	DcMotor shoulderSlide;
	DcMotor shoulderTilt;
	//DcMotor climbarmSlide;
	DcMotor intakeSpin;
	DcMotor shoulderPivot;
	//Servo climbarmServo;
	Servo shoulderServo;
	Servo intdropperServo;
	Servo rightZip;
	Servo leftZip;
	Servo Door;
	Servo boxTilt;


	/**
	 * Constructor
	 */
	public TeleOp() {

	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init() {


		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */

		motorRight1 = hardwareMap.dcMotor.get("motor_1R");
		motorLeft1 = hardwareMap.dcMotor.get("motor_1L");
		motorRight2 = hardwareMap.dcMotor.get("motor_2R");
		motorLeft2 = hardwareMap.dcMotor.get("motor_2L");
		motorLeft1.setDirection(DcMotor.Direction.REVERSE);
		motorLeft2.setDirection(DcMotor.Direction.REVERSE);
		shoulderSlide = hardwareMap.dcMotor.get("shoulder_slide");
		shoulderTilt= hardwareMap.dcMotor.get("shoulder_tilt");
		shoulderPivot = hardwareMap.dcMotor.get("shoulder_pivot");
		intakeSpin = hardwareMap.dcMotor.get("intake_spin");
		shoulderServo = hardwareMap.servo.get("shoulder_servo");
		intdropperServo = hardwareMap.servo.get("intdropper_servo");
		rightZip= hardwareMap.servo.get("zip_right");
		leftZip=hardwareMap.servo.get("zip_left");
		boxTilt=hardwareMap.servo.get("box_tilt");
		Door=hardwareMap.servo.get("door");


		// initilize servos
		intdropperServo.setPosition(0.5);
		leftZip.setPosition(0.8);
		rightZip.setPosition(0);
		boxTilt.setPosition(0.5);
		shoulderServo.setPosition(0.5);
		Door.setPosition(0.7);
	}
	/*
	 * This method will be called repeatedly in a loop
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {

		/*
		 * Gamepad 1
		 *
		 */

		// Left and right drive

		float left = gamepad1.left_stick_y;
		float right = gamepad1.right_stick_y;

		if (gamepad1.right_bumper) {
			rightZip.setPosition(0.1)	;
		}
		if (gamepad1.right_trigger > 0.8) {
			rightZip.setPosition(0.9);
		}
		if (gamepad1.left_bumper) {
			leftZip.setPosition(0.8)	;
		}
		if (gamepad1.left_trigger > 0.8) {
			leftZip.setPosition(0);
		}

		// This control the tilt of our intake servo.  Up down or stop


		if (gamepad1.y) {
			intdropperServo.setPosition(0.2); //up
			//clawPosition += clawDelta;
		}

		if (gamepad1.a) {
			intdropperServo.setPosition(0.8); //down
			//clawPosition -= clawDelta;
		}
		if (gamepad1.b) {
			intdropperServo.setPosition(0.5); //stop
			//clawPosition -= clawDelta;
		}


		/*
		* Send commands to the drive motors
		 */


		// clip the right/left values so that the values never exceed +/- 1
		right = Range.clip(right, -1, 1);
		left = Range.clip(left, -1, 1);

		// scale the joystick value to make it easier to control
		// the robot more precisely at slower speeds.
		right = (float) scaleInput(right);
		left = (float) scaleInput(left);

		// write the values to the motors
		motorRight1.setPower(right);
		motorRight2.setPower(right);
		motorLeft1.setPower(left);
		motorLeft2.setPower(left);


		/*
		 * Gamepad 2
		 *
		 */

		float shouldertilt = gamepad2.right_stick_y;

		double shoulder_slide_pow = 0;

		// command shoulder linear slide to go out
		if (gamepad2.right_bumper) {
			shoulder_slide_pow = 1;
		}
		// command shoulder linear slide to go in
		if (gamepad2.right_trigger > 0.5) {
			shoulder_slide_pow = -1;
		}

		shoulderSlide.setPower(shoulder_slide_pow);

		// run the spinner.

		double spinner = 0;

		if (gamepad2.x) {
			spinner = 1;
		}
		if (gamepad2.a) {
			spinner = -1;
		}
		if (gamepad2.y) {
			Door.setPosition(0.7); //close
		}
		if (gamepad2.b) {
			Door.setPosition(0.3); //open
		}
		intakeSpin.setPower(spinner);

		//tilt the box
		if (gamepad2.left_bumper) {
			boxTilt.setPosition(1)	;//up
		}
		if (gamepad2.left_trigger > 0.8) {
			boxTilt.setPosition(0.65);//down
		}

		// shoulder rotation left-right
		float rotshoulder = gamepad2.left_stick_x;
		if (rotshoulder > 0.8) {
			shoulderServo.setPosition(1.0);
		} else if (rotshoulder < -0.8){
			shoulderServo.setPosition(0.0);
		} else {
			shoulderServo.setPosition(0.5); // don't move
		}

//		float rotshouldery =  gamepad2.left_stick_y;
//		if (rotshouldery > 0.8) {
//			shoulderServo.setPosition(0.5);
//		}

		float power = shouldertilt;
		shoulderTilt.setPower(power);

		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
		telemetry.addData("Text", "*** Robot Data***");
		telemetry.addData("shoulder", "shoulder:  " + String.format("%.2f",rotshoulder));
		telemetry.addData("stilt", "stilt0:  " + String.format("%.2f", shouldertilt));
		telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", left));
		telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));

	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {

	}


	/*
	 * This method scales the joystick input so for low joystick values, the
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */
	double scaleInput(double dVal)  {
		double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
				0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * 16.0);

		// index should be positive.
		if (index < 0) {
			index = -index;
		}

		// index cannot exceed size of array minus 1.
		if (index > 16) {
			index = 16;
		}

		// get value from the array.
		double dScale = 0.0;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		// return scaled value.
		return dScale;
	}

}
