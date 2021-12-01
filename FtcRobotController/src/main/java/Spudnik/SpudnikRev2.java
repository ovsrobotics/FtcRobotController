/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
//Add speaker, spoiler, mudflaps (and reverse alarm), tinted PlexiGlass
package Spudnik;

import android.provider.CalendarContract;
import android.widget.Button;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.NavigableMap;




@TeleOp(name="SpudnikMaster (click this one)", group="Linear Opmode")
//@Disabled
public class SpudnikRev2 extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private DcMotor duck = null;
    private DcMotor armRaise = null;
    private DcMotor help = null;
    private Servo clampServo = null;
    private Servo spoiler = null;

    private DistanceSensor spinSensor = null;


    private BNO055IMU imu = null;

    private Orientation angles;
    private double curHeading;

    private Acceleration gravity;
    private double accX;
    private double accY;

    private Acceleration overall;
    private double overX;
    private double overY;

    private Position map;
    private double mapX;
    private double mapY;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Conprivate DcMotor raise = null;troller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        help = hardwareMap.get(DcMotor.class, "help");
        duck = hardwareMap.get(DcMotor.class, "spin");
        clampServo = hardwareMap.get(Servo.class, "clamp");
        armRaise = hardwareMap.get(DcMotor.class, "raise");
        spoiler = hardwareMap.get(Servo.class, "spoiler");
        spinSensor = hardwareMap.get(DistanceSensor.class, "spinSensor");



        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        help.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        duck.setDirection(DcMotor.Direction.FORWARD);
        armRaise.setDirection(DcMotorSimple.Direction.FORWARD);


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double spinPower; //power for duck spin
            double raisePower;

            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            boolean isClamp = true;

            double spinNum = spinSensor.getDistance(DistanceUnit.CM);
            // Comment out the method that's not used.  The default below is POV.


            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            checkOrientation();
            checkAcceleration();
            checkOverallAcceleration();
            checkNavigation();



            if(gamepad1.a) {
                spinPower = 100; //if a button is pressed, sets power to spin 100
                duck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            }
            else if(gamepad1.y) {
                spinPower = -100;
                duck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            else {
                spinPower = 0; //not pressing, power is off
                duck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                sleep(5);
            }


            isClamp = !(gamepad2.right_trigger >= .1) && !(gamepad1.right_trigger >= .1);

            if(isClamp) clampServo.setPosition(1); //sets position of servo
            else clampServo.setPosition(0);

            if(gamepad1.dpad_up || gamepad2.dpad_up) raisePower = 100;
            else if(gamepad1.dpad_down || gamepad2.dpad_down)raisePower = -100;
            else raisePower = 0;

            armRaise.setPower(raisePower);
            duck.setPower(spinPower);

            // leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            leftPower  = -gamepad1.left_stick_y ;
            rightPower = -gamepad1.right_stick_y ;
            // Send calculated power to wheels
            if(leftPower <= 0){
                spoiler.setPosition(0);

            }
            else spoiler.setPosition(1);

            if(gamepad1.dpad_down) help.setPower(-100);
            else if(gamepad1.dpad_up) help.setPower(100);
            else help.setPower(0);

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            // Show the elapsed HADPOWER (%.2f"), spinPower )game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Clamp", "Trigger1 (%.2f), Trigger2 (%.2f)", gamepad1.right_trigger, gamepad2.right_trigger);
            telemetry.addData("CarSpin", "Motor (%2f)", spinPower);
            telemetry.addData("ArmRaise", "Motor (%2f)", raisePower);
            telemetry.addData("SpinSense", "CM (%2f)", spinNum);
            telemetry.update();
        }
    }
    public void checkOrientation() {
        // read the orientation of the robot
        angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.imu.getPosition();
        // and save the heading
        curHeading = angles.firstAngle;
        telemetry.addData("Degrees", "* (%.2f)", curHeading);
    }
    public void checkAcceleration(){
        gravity = this.imu.getAcceleration();
        accX = gravity.xAccel;
        accY = gravity.yAccel;
        telemetry.addData("X - Y Acceleration", "X (%.2f), Y (%2f)", accX, accY);


    }
    public void checkOverallAcceleration(){
        overall = this.imu.getOverallAcceleration();
        overX = overall.xAccel;
        overY = overall.yAccel;
        telemetry.addData("X - Y Overall", "X (%.2f), Y (%2f)", overX, overY);

    }
    public void checkNavigation(){
        map = this.imu.getPosition();
        mapX = map.x;
        mapY = map.y;
        telemetry.addData("X - Y Map", "X (%2f), Y (%2f)", mapX, mapY);

    }
}
