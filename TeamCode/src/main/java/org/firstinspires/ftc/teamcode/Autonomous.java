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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.sql.Time;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group="Linear Opmode")
//@Disabled
public class Autonomous extends LinearOpMode {

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    private DcMotor slider = null;
    double slidePower;
    private Servo servo;
    double servoPower = 0.0;

    private int reverseControls = -1;

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
        slider = hardwareMap.get(DcMotor.class, "slider");
        servo = hardwareMap.servo.get("servo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        slider.setDirection((DcMotor.Direction.FORWARD));
        servo.setPosition(servoPower);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        moveStrafe(1, -1);
        moveForward(3, 1);
        moveStrafe(1, -1);

    }

    public void moveForward(long time, int direction){
        long start = System.currentTimeMillis();
        while (start<time) {
            start = System.currentTimeMillis();
            direction = 1;
            frontLeftDrive.setPower(direction);
            backLeftDrive.setPower(direction);
            frontRightDrive.setPower(direction);
            backRightDrive.setPower(direction);
        }
    }

    public void moveStrafe(long time, int direction){
        long start = System.currentTimeMillis();
        while (start<time) {
            start = System.currentTimeMillis();
            direction = 1;
            frontLeftDrive.setPower(direction);
            backLeftDrive.setPower(-direction);
            frontRightDrive.setPower(-direction);
            backRightDrive.setPower(direction);
        }
    }

    public void moveTurn(long time, int direction){
        long start = System.currentTimeMillis();
        while (start<time) {
            start = System.currentTimeMillis();
            direction = 1;
            frontLeftDrive.setPower(direction);
            backLeftDrive.setPower(direction);
            frontRightDrive.setPower(-direction);
            backRightDrive.setPower(-direction);
        }
    }




}
