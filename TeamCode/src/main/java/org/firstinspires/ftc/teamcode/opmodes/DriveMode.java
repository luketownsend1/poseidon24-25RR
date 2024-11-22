/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.HWProfile;

import java.util.Locale;

/** @noinspection ALL*/
@TeleOp(name="Drive", group="Robot")
//@Disabled
public class DriveMode extends LinearOpMode {

    private final static HWProfile robot = new HWProfile();

    @Override
    public void runOpMode() {
        /*
        These variables are private to the OpMode, and are used to control the drivetrain.
         */
        double left;
        double right;
        double forward;
        double rotate;
        double max;
        double poleToucherPosition = robot.POLE_DOWN;

        robot.init(hardwareMap, true);

        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        /* Wait for the game driver to press play */
        waitForStart();
        double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


        // Initializes ElapsedTimes. One for total runtime of the program and the others set up for toggles.
        ElapsedTime totalRuntime = new ElapsedTime();

        totalRuntime.reset();

        double denominator = 0;
        double frontLeftPower = 0;
        double backLeftPower = 0;
        double frontRightPower = 0;
        double backRightPower = 0;

        // booleans for keeping track of toggles
        boolean clawOpened = false;
        boolean clawRotated = true;
        boolean armRetracted = true;
        //boolean armClimb = false;



        /* Run until the driver presses stop */
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                robot.pinpoint.recalibrateIMU();
                //recalibrates the IMU without resetting position
            }

            robot.pinpoint.update();    //update the IMU value
            Pose2D pos = robot.pinpoint.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.RADIANS));
            telemetry.addData("Position", data);

            //botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            botHeading = pos.getHeading(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            frontLeftPower = (rotY + rotX + rx) / denominator;
            backLeftPower = (rotY - rotX + rx) / denominator;
            frontRightPower = (rotY - rotX - rx) / denominator;
            backRightPower = (rotY + rotX - rx) / denominator;

            robot.leftFrontDrive.setPower(frontLeftPower);
            robot.leftBackDrive.setPower(backLeftPower);
            robot.rightFrontDrive.setPower(frontRightPower);
            robot.rightBackDrive.setPower(backRightPower);



            //Rumble controller for endgame and flash controller light red
           /* if(totalRuntime.time() > 90 && totalRuntime.time()<90.25){
                gamepad1.rumble(50);
                gamepad1.setLedColor(255,0,0,50);
            } else if(totalRuntime.time() > 91 && totalRuntime.time()<91.25){
                gamepad1.rumble(50);
                gamepad1.setLedColor(255,0,0,50);
            } else if(totalRuntime.time() > 92 && totalRuntime.time()<92.25){
                gamepad1.rumble(50);
                gamepad1.setLedColor(255,0,0,50);
            } else if(totalRuntime.time() > 93) {
                gamepad1.setLedColor(255, 0, 0, 30000);
            }
           */

            telemetry.update();

        }
    }
}