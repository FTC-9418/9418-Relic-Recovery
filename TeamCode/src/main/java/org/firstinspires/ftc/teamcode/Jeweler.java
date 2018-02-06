package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class Jeweler extends LinearOpMode {

    private boolean lookForRed = false;
    private boolean isShort = false;
    private int color_threshold = 3;
    //private int hitBallDirection = Hardware.Direction_ReverseRight;

    public Jeweler(boolean lookForRed, boolean isShort) {
        this.lookForRed = lookForRed;
        this.isShort = isShort;
    }

    @Override
    public void runOpMode() {
        Hardware robot = new Hardware();
        robot.init(hardwareMap);

        // Set servos to starting values
        robot.startServo();

        // wait for the start button to be pressed.
        telemetry.addData("Mode: ", "waiting...");
        telemetry.update();

        double offset = 0.39;

        waitForStart();

        robot.sTs.setPosition(0.19);

        while (opModeIsActive() && offset > 0.35) {
            telemetry.addData("Red: ", robot.cs.red());
            telemetry.addData("Blu: ", robot.cs.blue());
            telemetry.update();
            if(hitJewel(robot, offset)) {
                break;
            }
            offset -= 0.005;
        }
        robot.stepServo(robot.fTb, 0.39);
        robot.stepServo(robot.sTs, 0.63);

        postJewel(robot);
    }

    public boolean hitJewel(Hardware robot, double init) {
        double pos = init;
        boolean detected = false;
        robot.sleep(500);
        if (lookForRed) {
            if (robot.cs.blue() > color_threshold) {
                pos = 0.01;
                detected = true;
            } else {
                if(robot.cs.red() > color_threshold) {
                    pos = 1;
                    detected = true;
                }
            }
        } else {
            if (robot.cs.red() > color_threshold) {
                pos = 0.01;
                detected = true;
            } else {
                if(robot.cs.blue() > color_threshold) {
                    pos = 1;
                    detected = true;
                }
            }
        }
        robot.fTb.setPosition(pos);
        if(detected) {
            robot.sleep(1000);
        }
        return detected;
    }

    private void postJewel(Hardware robot) {
        if(lookForRed) {
            robot.drive(Hardware.Direction_Forward, 0.7);
        } else {
            robot.drive(Hardware.Direction_Reverse, 0.7);
        }
        robot.sleep(1000);
        if(isShort) { // Robot goes to short cryptobox
            return;
        } else {  // Robot goes to far cryptobox
            robot.drive(Hardware.Direction_Forward, 0.7);
            robot.sleep(1000);
            robot.drive(Hardware.Direction_Left, 0.7);
            robot.sleep(500);
            if(lookForRed) {
                return;
            } else {
                robot.drive(Hardware.Direction_RotateLeft, 0.7);
                robot.sleep(500);
            }
        }
    }
}
