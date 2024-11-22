package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GoBilda.GoBildaPinpointDriver;

public class HWProfile {

    /* Declare OpMode members. */

    // Drive Motors //
    public DcMotor  leftFrontDrive   = null; //the left drivetrain motor
    public DcMotor  rightFrontDrive  = null; //the right drivetrain motor
    public DcMotor  leftBackDrive    = null;
    public DcMotor  rightBackDrive   = null;

    // Intake Hardware //
    public DcMotor intakeExtentionMotor = null;

    public Servo intakeJoint1Servo = null;
    public Servo intakeJoint2Servo = null;

    public Servo intakeClawRotationServo = null;

    public Servo intakeClawServo = null;

    // Depositing Hardware //
    public DcMotor depositExtentionMotor1 = null;
    public DcMotor depositExtentionMotor2 = null;

    public Servo depositJointServo = null;
    public Servo depositClawServo = null;


    public IMU      imu              = null;
    public GoBildaPinpointDriver pinpoint; // Declare OpMode member for the Odometry Computer
    public Servo    poleToucherServo = null;


    /* This constant is the number of encoder ticks for each degree of rotation of the arm.
    To find this, we first need to consider the total gear reduction powering our arm.
    First, we have an external 20t:100t (5:1) reduction created by two spur gears.
    But we also have an internal gear reduction in our motor.
    The motor we use for this arm is a 117RPM Yellow Jacket. Which has an internal gear
    reduction of ~50.9:1. (more precisely it is 250047/4913:1)
    We can multiply these two ratios together to get our final reduction of ~254.47:1.
    The motor's encoder counts 28 times per rotation. So in total you should see about 7125.16
    counts per rotation of the arm. We divide that by 360 to get the counts per degree. */
    public final double ELBOW_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation


    /* These constants hold the position that the arm is commanded to run to.
    These are relative to where the arm was located when you start the OpMode. So make sure the
    arm is reset to collapsed inside the robot before you start the program.

    In these variables you'll see a number in degrees, multiplied by the ticks per degree of the arm.
    This results in the number of encoder ticks the arm needs to move in order to achieve the ideal
    set position of the arm. For example, the ARM_SCORE_SAMPLE_IN_LOW is set to
    160 * ARM_TICKS_PER_DEGREE. This asks the arm to move 160° from the starting position.
    If you'd like it to move further, increase that number. If you'd like it to not move
    as far from the starting position, decrease it. */

    public final double ARM_COLLECT                = 0 * ELBOW_TICKS_PER_DEGREE;
    public final double ELBOW_HANG_CLIMB           = -10  * ELBOW_TICKS_PER_DEGREE;
    public final int    ELBOW_RESET                = 10;
    public final double ELBOW_HANG_ATTACH          = 135 * ELBOW_TICKS_PER_DEGREE;
    public final double ELBOW_COLLAPSED_INTO_ROBOT = 100;
    public final double ELBOW_SCORE_SAMPLE_IN_LOW = 105 * ELBOW_TICKS_PER_DEGREE;
    public final int    ELBOW_CLEAR_BARRIER        = 200;
//    public final double ARM_CLEAR_BARRIER         = 15 * ARM_TICKS_PER_DEGREE;
    //public final int    ELBOW_SCORE_SPECIMEN       = 300;
    public final double ELBOW_EXTENSION_ANGLE      = 400;
//    public final double ARM_SCORE_SPECIMEN        = 90 * ARM_TICKS_PER_DEGREE;
    public final int    ELBOW_TRAVERSE             = 600;
    public final double    ELBOW_HIGH_BASKET       = 100 * ELBOW_TICKS_PER_DEGREE;
    public final double ELBOW_SPECIMEN_PICKUP      = 14 * ELBOW_TICKS_PER_DEGREE;
    public final double ELBOW_SCORE_SPECIMEN       = 50  * ELBOW_TICKS_PER_DEGREE;



    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    public final double WRIST_FOLDED_IN   = 0.1667;
    public final double WRIST_FOLDED_OUT  = 0.5;

    /* A number in degrees that the triggers can adjust the arm position by */
    public final double FUDGE_FACTOR = 45 * ELBOW_TICKS_PER_DEGREE;

    public final double CLAW_OPEN = 0;
    public final double CLAW_CLOSED = .7;

    public final double EXTENSION_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;
    public final double EXTENSION_COLLAPSED = 0 * EXTENSION_TICKS_PER_MM;
    public final double EXTENSION_SCORING_IN_LOW_BASKET = 0 * EXTENSION_TICKS_PER_MM;
    public final double EXTENSION_SCORING_IN_HIGH_BASKET = 490 * EXTENSION_TICKS_PER_MM;
    public final int    EXTENSION_DOWN_MAX         = 1200;


    public final double POLE_DOWN = 0;
    public final double POLE_UP = 1;

    public Boolean opModeTeleop = null;

    /* local OpMode members. */
    HardwareMap hwMap =  null;


    public HWProfile() {

    }

    public void init(HardwareMap ahwMap, boolean teleOp) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        this.opModeTeleop = teleOp;

        if(opModeTeleop){
            /* Define and Initialize Motors */
            leftFrontDrive  = hwMap.dcMotor.get("frontLeftMotor");
            leftBackDrive   = hwMap.dcMotor.get("backLeftMotor");
            rightFrontDrive = hwMap.dcMotor.get("frontRightMotor");
            rightBackDrive  = hwMap.dcMotor.get("backRightMotor");

           /*
           we need to reverse the left side of the drivetrain so it doesn't turn when we ask all the
           drive motors to go forward.
            */

            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
            much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
            stops much quicker. */
            leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            // Retrieve the IMU from the hardware map
            imu  = hwMap.get(IMU.class, "imu");
            // Adjust the orientation parameters to match your robot
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
            imu.initialize(parameters);
            pinpoint = hwMap.get(GoBildaPinpointDriver.class,"pinpoint");
            pinpoint.resetPosAndIMU();


        }

        // Intake Hardware Declaration //

        // Motors
        intakeExtentionMotor = hwMap.dcMotor.get("intakeExtend");
        intakeExtentionMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeExtentionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeExtentionMotor.setTargetPosition(0);
        intakeExtentionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeExtentionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Servos
        intakeJoint1Servo = hwMap.servo.get("intakeJoint1");
        intakeJoint2Servo = hwMap.servo.get("intakeJoint2");

        intakeClawServo = hwMap.servo.get("intakeClaw");

        intakeClawRotationServo = hwMap.servo.get("intakeClawRotate");

        // Deposit Hardware Declaration //

        // Motors
        depositExtentionMotor1 = hwMap.dcMotor.get("depositExtend1");
        depositExtentionMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        depositExtentionMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        depositExtentionMotor1.setTargetPosition(0);
        depositExtentionMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        depositExtentionMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        depositExtentionMotor2 = hwMap.dcMotor.get("depositExtend2");
        depositExtentionMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        depositExtentionMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        depositExtentionMotor2.setTargetPosition(0);
        depositExtentionMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        depositExtentionMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Servos
        depositJointServo = hwMap.servo.get("depositJoint");

        depositClawServo = hwMap.servo.get("depositClaw");


        // Halt all motors, just to make sure.
        intakeExtentionMotor.setPower(0);
        depositExtentionMotor1.setPower(0);
        depositExtentionMotor2.setPower(0);

        poleToucherServo = hwMap.get(Servo.class, "poleToucher");


    }
}
