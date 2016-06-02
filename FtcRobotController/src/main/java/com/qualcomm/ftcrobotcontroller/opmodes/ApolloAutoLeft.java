
package com.qualcomm.ftcrobotcontroller.opmodes;

import android.util.Log;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;

/**
 * Created by maddi_000 on 2/6/2016.
 */
public class ApolloAutoLeft extends LinearOpMode {
    /* This is the port on the Core Device Interface Module        */
    /* in which the navX-Model Device is connected.  Modify this  */
    /* depending upon which I2C port you are using.               */
    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navx_device;
    private navXPIDController yawPIDController;
    private ElapsedTime runtime = new ElapsedTime();

    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    private  double TARGET_ANGLE_DEGREES = 0.0;
    private final double TOLERANCE_DEGREES = 4.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.01; // was 0.005
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;

    private boolean calibration_complete = false;
    public double limit(double a) {
        return Math.min(Math.max(a, MIN_MOTOR_OUTPUT_VALUE), MAX_MOTOR_OUTPUT_VALUE);
    }

    // Declare the deviced we are using

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

    @Override
    public void runOpMode() throws InterruptedException {
        //Map all fo the hardware devices into our variables

        // Gyro motion sensor
        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

        motorRight1 = hardwareMap.dcMotor.get("motor_1R");
        motorLeft1 = hardwareMap.dcMotor.get("motor_1L");
        motorRight2 = hardwareMap.dcMotor.get("motor_2R");
        motorLeft2 = hardwareMap.dcMotor.get("motor_2L");
        motorLeft1.setDirection(DcMotor.Direction.REVERSE);
        motorLeft2.setDirection(DcMotor.Direction.REVERSE);

        motorLeft1.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorRight1.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorLeft2.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorRight2.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        waitOneFullHardwareCycle();
        motorLeft1.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorRight1.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorLeft2.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorRight2.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        shoulderSlide = hardwareMap.dcMotor.get("shoulder_slide");
        shoulderTilt = hardwareMap.dcMotor.get("shoulder_tilt");
        shoulderPivot = hardwareMap.dcMotor.get("shoulder_pivot");
        //climbarmSlide = hardwareMap.dcMotor.get("climbarm_slide");
        intakeSpin = hardwareMap.dcMotor.get("intake_spin");
        shoulderServo = hardwareMap.servo.get("shoulder_servo");
        intdropperServo = hardwareMap.servo.get("intdropper_servo");
        rightZip = hardwareMap.servo.get("zip_right");
        leftZip = hardwareMap.servo.get("zip_left");
        boxTilt = hardwareMap.servo.get("box_tilt");
        Door = hardwareMap.servo.get("door");

        // initilize servos
        intdropperServo.setPosition(0.5);
        leftZip.setPosition(0.8);
        rightZip.setPosition(0);
        boxTilt.setPosition(0.5);
        shoulderServo.setPosition(0.5);
        Door.setPosition(0.7);

        /* If possible, use encoders when driving, as it results in more */
        /* predictable drive system response.                           */
        //leftMotor.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //rightMotor.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        /* Create a PID Controller which uses the Yaw Angle as input. */
        yawPIDController = new navXPIDController(navx_device,
                navXPIDController.navXTimestampedDataSource.YAW);

        /* Configure the PID controller */
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);
        waitForStart();


        while ( !calibration_complete ) {
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
            calibration_complete = !navx_device.isCalibrating();
            if (!calibration_complete) {
                telemetry.addData("navX-Micro", "Startup Calibration in Progress");
            }
        }
        navx_device.zeroYaw();

        /* Wait for new Yaw PID output values, then update the motors
           with the new PID value with each new output value.
         */

        final double TOTAL_RUN_TIME_SECONDS = 30.0;
        int DEVICE_TIMEOUT_MS = 500;
        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

        /* Drive straight forward at 1/2 of full drive speed */
        double drive_speed = 0.5;

        DecimalFormat df = new DecimalFormat("#.##");

// Some auto commands:
// DriveOnHeading(navXPIDController.PIDResult yawPIDResult, float heading, float distanceInches)
// DriveOnHeadingReverse(navXPIDController.PIDResult yawPIDResult, float heading, float distanceInches)
// StopDriving()
// TurnToHeading(navXPIDController.PIDResult yawPIDResult, float heading, double maxTimeSeconds)


        // start of actual program that does something
        try {
            //lower intake
            intdropperServo.setPosition(0.8); //down
            sleep(8000);
            intdropperServo.setPosition(0.5); //stop
            float heading = 0;
            float distanceInches=36;
            intakeSpin.setPower(-1);
            DriveOnHeading(yawPIDResult, heading, distanceInches);
            StopDriving();
            Log.w("AutoLog", "1");
            TurnToHeading(yawPIDResult, (float) 45., (double) 3.);
            StopDriving();
            Log.w("AutoLog", "2");
            sleep(2000);
            heading = 45;
            distanceInches=24;
            DriveOnHeadingReverse(yawPIDResult, heading, distanceInches);
            StopDriving();
            intakeSpin.setPower(0);

        }
        catch(InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
        finally {
            StopDriving();
            navx_device.close();
            telemetry.addData("LinearOp", "Complete");
        }
    }



    public void DriveOnHeading(navXPIDController.PIDResult yawPIDResult, float heading, float distanceInches) {

        // calculate encoder counts for distance
        float wheelDiameter = 6; // inches
        float wheelCirc = wheelDiameter* (float) 3.14159;
        float encoderTicksPerRotation = 1860;
        float ticksPerInch =encoderTicksPerRotation/wheelCirc;
        int ticksToTravel=(int) (distanceInches*ticksPerInch);

        // check motor position
        int startEncCount=motorLeft1.getCurrentPosition();
        int DEVICE_TIMEOUT_MS = 500;
        /* Drive straight forward at 1/2 of full drive speed */
        double drive_speed = 0.5;
        yawPIDController.setSetpoint(heading);
        DecimalFormat df = new DecimalFormat("#.##");
        try {
            while ((motorLeft1.getCurrentPosition()-startEncCount) > -ticksToTravel &&
                    !Thread.currentThread().isInterrupted()) {
                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    if (yawPIDResult.isOnTarget()) {
                        Drive(-drive_speed, -drive_speed);
                        telemetry.addData("PIDOutput", df.format(drive_speed) + ", " +
                                df.format(drive_speed));
                    } else {
                        double output = yawPIDResult.getOutput();
                        if (output < -0.5) {
                            output = -0.5;
                        }
                        if (output > 0.5) {
                            output = 0.5;
                        }
                        Drive(-drive_speed - output, -drive_speed + output);
                        telemetry.addData("PIDOutput", df.format(limit(-drive_speed - output)) + ", " +
                                df.format(limit(-drive_speed + output)));
                    }
                    telemetry.addData("Yaw", df.format(navx_device.getYaw()));
                    telemetry.addData("Output", df.format(yawPIDResult.getOutput()));
                    telemetry.addData("Enc:", motorLeft1.getCurrentPosition());
                    telemetry.addData("EncStart:", startEncCount);
                } else {
			        /* A timeout occurred */
                    Log.w("navXDriveStraightOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                }
            }
        }
        catch(InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }

    public void DriveOnHeadingReverse(navXPIDController.PIDResult yawPIDResult, float heading, float distanceInches) {

        // calculate encoder counts for distance
        float wheelDiameter = 6; // inches
        float wheelCirc = wheelDiameter* (float) 3.14159;
        float encoderTicksPerRotation = 1860;
        float ticksPerInch =encoderTicksPerRotation/wheelCirc;
        int ticksToTravel=(int) (distanceInches*ticksPerInch);

        // check motor position
        int startEncCount=motorLeft1.getCurrentPosition();
        int DEVICE_TIMEOUT_MS = 500;
        /* Drive straight forward at 1/2 of full drive speed */
        double drive_speed = 0.5;
        yawPIDController.setSetpoint(heading);
        DecimalFormat df = new DecimalFormat("#.##");
        try {
            while ((motorLeft1.getCurrentPosition()-startEncCount) < ticksToTravel &&
                    !Thread.currentThread().isInterrupted()) {
                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    if (yawPIDResult.isOnTarget()) {
                        Drive(drive_speed, drive_speed);
                        telemetry.addData("PIDOutput", df.format(drive_speed) + ", " +
                                df.format(drive_speed));
                    } else {
                        double output = yawPIDResult.getOutput();
                        if (output < -0.5) {
                            output = -0.5;
                        }
                        if (output > 0.5) {
                            output = 0.5;
                        }
                        Drive(drive_speed - output, drive_speed + output);

                        telemetry.addData("PIDOutput", df.format(limit(-drive_speed - output)) + ", " +
                                df.format(limit(-drive_speed + output)));
                    }
                    telemetry.addData("Yaw", df.format(navx_device.getYaw()));
                    telemetry.addData("Output", df.format(yawPIDResult.getOutput()));
                    telemetry.addData("RevEnc:", motorLeft1.getCurrentPosition());
                    telemetry.addData("EncStart:", startEncCount);
                } else {
			        /* A timeout occurred */
                    Log.w("navXDriveStraightOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                }
            }
        }
        catch(InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }


    public void StopDriving() {
        telemetry.addData("Stopping, Enc:", motorLeft1.getCurrentPosition());
        Drive(0,0);
    }
    public void Drive(double left, double right) {
        motorLeft1.setPower(left);
        motorLeft2.setPower(left);
        motorRight1.setPower(right);
        motorRight2.setPower(right);
    }

    public void TurnToHeading(navXPIDController.PIDResult yawPIDResult, float heading, double maxTimeSeconds) {
        try {
            long starttime= System.currentTimeMillis();

        TARGET_ANGLE_DEGREES = heading;
        int DEVICE_TIMEOUT_MS = 500;

        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        DecimalFormat df = new DecimalFormat("#.##");

        while (
                !Thread.currentThread().isInterrupted()&& (System.currentTimeMillis()-starttime) < (long) (maxTimeSeconds*1000) ) {
            if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                if (yawPIDResult.isOnTarget()) {
                    float zero = 0;
                    motorLeft1.setPowerFloat();
                    motorRight1.setPowerFloat();
                    motorLeft2.setPowerFloat();
                    motorRight2.setPowerFloat();
                    telemetry.addData("PIDOutput", df.format(0.00));
                } else {
                    double output = -yawPIDResult.getOutput();
                    motorLeft1.setPower(output);
                    motorLeft2.setPower(output);
                    motorRight1.setPower(-output);
                    motorRight2.setPower(-output);
                    telemetry.addData("PIDOutput", df.format(output) + ", " +
                            df.format(-output));
                }
            } else {
			    /* A timeout occurred */
                Log.w("navXRotateOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
            }
            telemetry.addData("Yaw", df.format(navx_device.getYaw()));
        }
    }
        // && (Math.abs(heading-navx_device.getYaw()) > TOLERANCE_DEGREES)
    catch(InterruptedException ex) {
        Thread.currentThread().interrupt();
    }

    }
}
