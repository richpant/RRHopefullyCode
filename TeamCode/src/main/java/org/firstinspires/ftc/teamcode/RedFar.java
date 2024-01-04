
/* Copyright (c) 2023 FIRST. All rights reserved.
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

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
 * driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * The code assumes a Robot Configuration with motors named: leftfront_drive and rightfront_drive, leftback_drive and rightback_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID (usually starting at 0)
 *
 * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
 * Manually drive the robot until it displays Target data on the Driver Station.
 *
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */

@Autonomous(name="RedFar")

public class RedFar extends LinearOpMode{

    private final int READ_PERIOD = 1;


    public int pixelspot = 2;
    private DcMotor lift            = null;
    private DcMotor gear      = null; //e1

    private DcMotor hanger  = null;  //e3

    private Servo pivot = null; //es1

    private Servo droneServo    = null; //es3
    private Servo clawL       = null; //es1
    private Servo clawR       = null; //es2





    private HuskyLens huskyLens;
    //TODO add your other motors and sensors here


    @Override public void runOpMode() {
        Pose2d beginPose = new Pose2d(60, -30, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
               lift = hardwareMap.get(DcMotor.class, "lift");
        gear = hardwareMap.get(DcMotor.class, "gear");
        //intake      = hardwareMap.get(DcMotor.class, "intake");
        hanger      = hardwareMap.get(DcMotor.class, "hanger");
        //intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        pivot = hardwareMap.get(Servo.class, "pivot");
        //claw = hardwareMap.get(Servo.class, "claw");
        droneServo  = hardwareMap.get(Servo.class, "droneServo");
        clawL = hardwareMap.get(Servo.class, "clawL");
        clawR = hardwareMap.get(Servo.class, "clawR");

        // pivot.setPosition(.25);//1
        clawL.setPosition(.33);//0
        clawR.setPosition(.37);//0
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");

        //TODO initialize the sensors and motors you added
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        gear.setDirection(DcMotor.Direction.REVERSE);
        gear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS); //from huskylens example
        rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }//makes sure the huskylens is talking to the control hub
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);// can change to other algorithms


        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {

            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();// from huskylens
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());// this gives you the data
                telemetry.addData("location?", blocks[i].x);// this gives you just x
                //TODO ensure your x values of the husky lens are appropriate to the desired areas
                //----------------------------3----------------------------\\
                if (blocks[i].x > 210) {
                    Actions.runBlocking(
                            drive.actionBuilder(beginPose)
                                    .stopAndAdd(drive.up())//lower pivot
                                    .waitSeconds(.5)
                                    .stopAndAdd(geardown())//arm down
                                    .waitSeconds(.5)
                                    .setTangent(180)
                                    .splineTo(new Vector2d(26,-30.5),2*Math.PI/3)//drive to spike mark
                                    .stopAndAdd(drive.openL())//score purple
                                    .waitSeconds(.5)
                                    .stopAndAdd(drive.gearupABit())
                                    .waitSeconds(.5)
                                    .lineToYLinearHeading(-36,2*Math.PI/3)//back up
                                    .waitSeconds(.5)
                                    .stopAndAdd(drive.pivotPickUp())
                                   // .splineToSplineHeading(new Pose2d(-12,-48,0),Math.toRadians(180))
                                    .splineTo(new Vector2d(7.5, -47),Math.toRadians(270))//line up with white stack
                                    .waitSeconds(.5)
                                    .lineToYConstantHeading(-51.5)//forward into white
                                    .waitSeconds(.5)
                                    .stopAndAdd(drive.closeL())//pick up white
                                    .waitSeconds(.5)
                                    .lineToY(-40)
                                    .strafeTo(new Vector2d(8,-40))//line up to go back
                                    .waitSeconds(.5)
                                    .lineToYConstantHeading(53)//drive to backboard
                                    .strafeTo(new Vector2d(26.5,53))//strafe to score
                                    .waitSeconds(.5)
                                    .stopAndAdd(liftInHere())
                                    .stopAndAdd(flipToScore())
                                    .waitSeconds(.5)
                                    .lineToYConstantHeading(59.5)//back all the way up
                                    .waitSeconds(.75)
                                    .stopAndAdd(drive.openL())//score white
                                    .waitSeconds(.5)
                                    .strafeTo(new Vector2d(42,59))
                                    .waitSeconds(.5)
                                    .stopAndAdd(drive.openR())//score yellow
                                    .waitSeconds(.5)
                                    .lineToYConstantHeading(57)
                                    .stopAndAdd(drive.up())
                                    .stopAndAdd(gearend())
                                    .stopAndAdd(liftIn())
                                    .build());
                                    sleep(400000);



                }
                //----------------------------2----------------------------\\
                if (blocks[i].x > 100 && blocks[i].x < 200) {
                    Actions.runBlocking(
                            drive.actionBuilder(beginPose)
                                    .stopAndAdd(drive.up())//lower pivot
                                    .waitSeconds(.5)
                                    .stopAndAdd(geardown())//arm down
                                    .waitSeconds(.5)
                                    .setTangent(0)
                                    .strafeTo(new Vector2d(35.5, -27.5))
                                    .stopAndAdd(drive.openL())//score purple
                                    .waitSeconds(.5)
                                    .strafeTo(new Vector2d(54, -32))//back away from purple
                                    .waitSeconds(.5)
                                    .stopAndAdd(drive.gearupABit())
                                    .waitSeconds(.5)
                                    .stopAndAdd(drive.pivotPickUp())
                                    // .splineToSplineHeading(new Pose2d(-12,-48,0),Math.toRadians(180))
                                    .splineTo(new Vector2d(30, -48), Math.toRadians(270))//line up with white stack
                                    .waitSeconds(.5)
                                    .lineToYConstantHeading(-51.5)//forward into white
                                    .waitSeconds(.5)
                                    .stopAndAdd(drive.closeL())//pick up white
                                    .waitSeconds(.5)
                                    .lineToY(-44)
                                    .strafeTo(new Vector2d(7.5, -44))//line up to go back
                                    .waitSeconds(.5)
                                    .lineToYConstantHeading(53)//drive to backboard
                                    .strafeTo(new Vector2d(34.5, 53))//strafe to score
                                    .waitSeconds(.5)
                                    .stopAndAdd(liftInHere2())
                                    .stopAndAdd(flipToScore2())
                                    .waitSeconds(.5)
                                    .lineToYConstantHeading(59)//back all the way up
                                    .waitSeconds(1.2)
                                    .stopAndAdd(drive.openR())//score yellow
                                    .waitSeconds(.5)
                                    .strafeTo(new Vector2d(38, 59))
                                    .waitSeconds(.5)
                                    .stopAndAdd(drive.openL())//score white
                                    .waitSeconds(.5)
                                    .lineToYConstantHeading(57)
                                    .stopAndAdd(drive.up())
                                    .stopAndAdd(gearend())
                                    .stopAndAdd(liftIn())
                                    .build());
                    sleep(400000);
                }

                //----------------------------1----------------------------\\
                if (blocks[i].x < 100) {
                    Actions.runBlocking(
                            drive.actionBuilder(beginPose)
                                    .stopAndAdd(drive.up())//lower pivot
                                    .waitSeconds(.5)
                                    .stopAndAdd(geardown())//arm down
                                    .waitSeconds(.5)
                                    .strafeTo(new Vector2d(42,-40.5))
                                    .stopAndAdd(drive.openL())//score purple
                                    .waitSeconds(.5)
                                    .stopAndAdd(drive.gearupABit())
                                    .waitSeconds(.5)
                                    .strafeTo(new Vector2d(54,-27))
                                    .waitSeconds(.5)
                                    .stopAndAdd(drive.pivotPickUp())
                                    // .splineToSplineHeading(new Pose2d(-12,-48,0),Math.toRadians(180))
                                    .splineTo(new Vector2d(7.5, -45),Math.toRadians(270))//line up with white stack
                                    .waitSeconds(.5)
                                    .lineToYConstantHeading(-51.5)//forward into white
                                    .waitSeconds(.5)
                                    .stopAndAdd(drive.closeL())//pick up white
                                    .waitSeconds(.5)
                                    .lineToY(-40)
                                    .strafeTo(new Vector2d(7.5,-40))//line up to go back
                                    .waitSeconds(.5)
                                    .lineToYConstantHeading(53)//drive to backboard
                                    .strafeTo(new Vector2d(27,53))//strafe to score
                                    .waitSeconds(.5)
                                    .stopAndAdd(liftInHere3())
                                    .stopAndAdd(flipToScore())
                                    .waitSeconds(.5)
                                    .lineToYConstantHeading(59)//back all the way up
                                    .waitSeconds(1)
                                    .stopAndAdd(drive.openR())//score Yellow
                                    .waitSeconds(.5)
                                    .strafeTo(new Vector2d(36,59))
                                    .waitSeconds(1)
                                    .stopAndAdd(drive.openL())//score White
                                    .waitSeconds(.5)
                                    .lineToYConstantHeading(57)
                                    .stopAndAdd(drive.up())
                                    .stopAndAdd(gearend())
                                    .stopAndAdd(liftIn())
                                    .build());
                    sleep(400000);
                }

            }
        }
    }
    public Action geardown(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                gear.setTargetPosition(-400);
                gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gear.setPower(0.333);

                while (gear.isBusy()) {
                    sleep(25);
                }

                gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                gear.setPower(0); return false;
            }
        };
    }
    public Action liftInHere(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setTargetPosition(-700);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(0.7);
                return false;
            }
        };
    }
    public Action flipToScore(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                pivot.setPosition(0.21);
                gear.setTargetPosition(750);
                gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gear.setPower(0.15);

                return false;
            }
        };
    }


    public Action gearend(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                gear.setTargetPosition(-600);
                gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gear.setPower(0.333);

                while (gear.isBusy()) {
                    sleep(25);
                }

                gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                gear.setPower(0); return false;
            }
        };
    }
    public Action liftIn(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setTargetPosition(700);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(0.7);
                return false;
            }
        };
    }
    public Action flipToScore2(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                pivot.setPosition(0.21);
                gear.setTargetPosition(750);
                gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gear.setPower(0.1);

                return false;
            }
        };
    }
    public Action liftInHere2(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setTargetPosition(-850);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(0.7);
                return false;
            }
        };
    }
    public Action liftInHere3(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setTargetPosition(-850);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(0.7);
                return false;
            }
        };
    }










}