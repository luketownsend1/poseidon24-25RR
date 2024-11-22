package org.firstinspires.ftc.teamcode.libraries;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.CSAutoParams;
import org.firstinspires.ftc.teamcode.hardware.HWProfile;

public class RRMechOps {

    public HWProfile robot;
    public LinearOpMode opMode;
    public CSAutoParams params;

    /*
     * Constructor
     */
    public RRMechOps(HWProfile myRobot, LinearOpMode myOpMode, CSAutoParams autoParams){
        robot = myRobot;
        opMode = myOpMode;
        params = autoParams;

    }   // close RRMechOps constructor

    /*
     * ###########################################################################################
     * ###########################################################################################
     * #######################      SYSTEM CONTROLS      #########################################
     * ###########################################################################################
     * ###########################################################################################
     */

    public void poleTouch(){
        robot.poleToucherServo.setPosition(robot.POLE_UP);
    }



    /***
     * This method provides a delay that will cancel if the program is stopped
     * @param time
     */
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();

        // wait while a stop is not requested
        while (!opMode.isStopRequested() && timer.time() < time) {
        }
    }

}   // close the RRMechOps class