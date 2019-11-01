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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
//@Disabled
public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();

    //Drive motors for mecanum wheels
    // 152 rpm
    private DcMotor frontLeftDrive = null;
    // 152 rpm
    private DcMotor frontRightDrive = null;
    // 152 rpm
    private DcMotor backLeftDrive = null;
    // 152 rpm
    private DcMotor backRightDrive = null;

    private double motorWeight = 0.65789473684;

    //Two intake motors
    private DcMotor leftIntake = null;
    private DcMotor rightIntake = null;

    //Grabber arm motor and power
    private DcMotor grabberArm = null;
    private double grabberPower = 1;

    //Linear lift motor
    private DcMotor linLift = null;

    //Servo for hooking block from side
    private Servo servo;
    //Servo for grabbing with arm
    private Servo grabServo;

    //Two servos for linear lift chopsticks
    private Servo leftLiftServo;
    private Servo rightLiftServo;

    //Reverse controls modifier
    private double reverseControls = 1;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        leftIntake = hardwareMap.get(DcMotor.class, "left_intake");
        rightIntake = hardwareMap.get(DcMotor.class, "right_intake");

        linLift = hardwareMap.get(DcMotor.class, "lin_lift");

        grabberArm = hardwareMap.get(DcMotor.class, "grabber_arm");
        servo = hardwareMap.servo.get("servo");
        grabServo = hardwareMap.servo.get("grab_servo");
        leftLiftServo = hardwareMap.servo.get("left_lift_servo");
        rightLiftServo = hardwareMap.servo.get("right_lift_servo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftIntake.setDirection(DcMotor.Direction.FORWARD);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);

        grabberArm.setDirection((DcMotor.Direction.FORWARD));
        linLift.setDirection((DcMotor.Direction.FORWARD));
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // setup the inputs
            double G1LeftStickY = reverseControls * gamepad1.left_stick_y;
            double G1RightStickY = reverseControls * gamepad1.right_stick_y;
            double G1LeftStickX = -reverseControls * gamepad1.left_stick_x;
            double G1RightStickX = -reverseControls * gamepad1.right_stick_x;
            boolean G1RightBumper = gamepad1.right_bumper;
            boolean G1LeftBumper = gamepad1.left_bumper;

            double G2LeftStickY = gamepad2.left_stick_y * 0.6;
            double G2RightStickY = gamepad2.right_stick_y;

            double G2RightTrigger = gamepad2.right_trigger;
            double G2LeftTrigger = -gamepad2.left_trigger;

            // strafe  Mode
            frontLeftDrive.setPower(G1LeftStickY - G1LeftStickX);
            backLeftDrive.setPower((G1LeftStickY + G1LeftStickX));
            frontRightDrive.setPower(G1RightStickY + G1LeftStickX);
            backRightDrive.setPower((G1RightStickY - G1LeftStickX));

            //Stops arm from falling with gravity
            if(gamepad2.right_bumper){
                grabberPower = -0.3;
            }
            else{
                grabberPower = 1;
            }

            //Sets arm power with triggers
            grabberArm.setPower((G2LeftTrigger+G2RightTrigger)*0.7);
            //Sets linlift power to right stick
            linLift.setPower(G2RightStickY);

            //grabs block on a and releases on b
            if(gamepad2.a){
                grabServo.setPosition(1);
            }
            if(gamepad2.b){
                grabServo.setPosition(0.8);
            }

            if(gamepad2.x){
                //lin lift servo close
                leftLiftServo.setPosition(0.5);
                rightLiftServo.setPosition(-0.5);
            }
            if(gamepad2.y){
                //lin lift servo open
                rightLiftServo.setPosition(1);
                leftLiftServo.setPosition(-1);
            }

            //side block servo open and close
            if(gamepad1.a){
                servo.setPosition(0.6);
            }
            else if(gamepad1.b){
                servo.setPosition(0);
            }

            //Starts intake motors and lifts grabber arm
            if (gamepad1.right_bumper){
                leftIntake.setPower(-1);
                rightIntake.setPower(-1);
                grabberArm.setPower(0.7);
            }
            if (gamepad1.left_trigger > 0){
                leftIntake.setPower(1);
                rightIntake.setPower(1);
                grabberArm.setPower(0.7);
            }
            if (gamepad1.left_bumper){
                leftIntake.setPower(0);
                rightIntake.setPower(0);
                grabberArm.setPower(0);
            }




            // Change the control direction with the x button
            if (gamepad1.x){
                reverseControls = -reverseControls;
                telemetry.addData("Controls Status", "Direction: " + reverseControls);
            }


            //Adds slowing modifier to controls when y is pressed and removes when y is pressed again
            if (gamepad1.y){
                if(reverseControls == 1){
                    reverseControls = reverseControls*0.5;
                    telemetry.addData("Controls Status", "SLOW MODE");
                }
                else if(reverseControls == 0.5){
                    reverseControls = reverseControls*2;
                    telemetry.addData("Controls Status", "FAST MODE");
                }
                telemetry.update();
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Motors", "left (%.2f), right (%.2f)", data);
            telemetry.update();
        }
    }
}
