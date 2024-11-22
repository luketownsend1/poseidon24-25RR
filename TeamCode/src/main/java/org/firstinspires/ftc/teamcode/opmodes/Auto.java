package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.hardware.HWProfile;
import org.firstinspires.ftc.teamcode.libraries.RobotLibrary;

@Autonomous(name = "Auto", group = "Robot")
//@Disabled

public class Auto extends LinearOpMode {
    private HWProfile robot = new HWProfile();
    private final LinearOpMode opMode = this;
    private RobotLibrary lib = new RobotLibrary(robot, opMode);

    public Auto() {

    }

    public void runOpMode() {
        telemetry.addData("Status: ", "Initializing...");
        telemetry.update();

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        robot.init(hardwareMap, true);

        telemetry.addData("Status: ","Ready.");
        telemetry.update();

        waitForStart();
        telemetry.addData("Status: ","Charge!!");
        telemetry.update();



        requestOpModeStop();
        telemetry.addData("Status: ","Program Completed.");
        telemetry.update();
    }
}
