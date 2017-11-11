package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class Jeweler extends LinearOpMode {

    private boolean lookForRed = false;
    private int color_threshold = 3;
    private int hitBallDirection = Hardware.Direction_ReverseRight;

    public Jeweler(boolean lookForRed) {
        this.lookForRed = lookForRed;
        if (lookForRed){
            //this.hitBallDirection = Hardware.hitRedJewel();
        }
    }

    @Override
    public void runOpMode() {
        Hardware robot = new Hardware();
        robot.init(hardwareMap);

        // wait for the start button to be pressed.
        telemetry.addData("Mode ", "waiting...");
        telemetry.update();


        waitForStart();

        telemetry.update();

        robot.sTs.setPosition(0.2);
    }

    public void hitball(Hardware robot) {
        if (lookForRed) {
            if (robot.cs.blue() > color_threshold) {
                robot.fTb.setPosition(0.7);
            } else {
                robot.fTb.setPosition(-0.7);
            }
        } else {
            if (robot.cs.blue() > color_threshold) {
                robot.fTb.setPosition(-0.7);
            } else {
                robot.fTb.setPosition(0.7);
            }
        }
        robot.naturalServo();
        robot.stop();
    }
}
