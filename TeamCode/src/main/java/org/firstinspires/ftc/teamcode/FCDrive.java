package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="FCDrive")
public class FCDrive extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lF        = null; //c0
    private DcMotor rF        = null; //c1
    private DcMotor lB        = null; //c2
    private DcMotor rB        = null; //c3
    private DcMotor lift      = null; //e0
    private DcMotor gear      = null; //e1

    private Servo pivot       = null; //es0
    private Servo clawL       = null; //es1
    private Servo clawR       = null; //es2

    private Servo droneServo  = null; //es5

    @Override
    public void runOpMode() {
        lF = hardwareMap.get(DcMotor.class, "lF");
        rF = hardwareMap.get(DcMotor.class, "rF");
        lB = hardwareMap.get(DcMotor.class, "lB");
        rB = hardwareMap.get(DcMotor.class, "rB");
        lift = hardwareMap.get(DcMotor.class, "lift");
        gear = hardwareMap.get(DcMotor.class, "gear");
        pivot = hardwareMap.get(Servo.class, "pivot");
        clawL = hardwareMap.get(Servo.class, "clawL");
        clawR = hardwareMap.get(Servo.class, "clawR");
        droneServo = hardwareMap.get(Servo.class, "droneServo");
        clawL.setPosition(.45);
        clawR.setPosition(.25);
        //pivot.setPosition(0);

        droneServo.setPosition(0.5);
        lF.setDirection(DcMotor.Direction.REVERSE);
        lB.setDirection(DcMotor.Direction.REVERSE);
        //rF.setDirection(DcMotor.Direction.REVERSE);
        //rB.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.REVERSE);
        gear.setDirection(DcMotor.Direction.REVERSE);
        //hanger.setDirection(DcMotor.Direction.REVERSE);

        gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        gear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gear.setTargetPosition(0);
        lift.setTargetPosition(0);
        pivot.setPosition(.8);
        clawL.setPosition(.45);
        clawR.setPosition(.25);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        boolean clawState = false;
        boolean oldArmButton = true;

        //----------------------------Main-Code----------------------------\\

        while (opModeIsActive()) {

            //----------------------------Mecanum-Drive-Code----------------------------\\

            double max;
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            double lFPower = axial + lateral + yaw;
            double rFPower = axial - lateral - yaw;
            double lBPower = axial - lateral + yaw;
            double rBPower = axial + lateral - yaw;
            max = Math.max(Math.abs(lFPower), Math.abs(rFPower));
            max = Math.max(max, Math.abs(lBPower));
            max = Math.max(max, Math.abs(rBPower));
            if (max > 1.0) {
                lFPower /= max;
                rFPower /= max;
                lBPower /= max;
                rBPower /= max;
            }

            // Retrieve the IMU from the hardware map
            IMU imu = hardwareMap.get(IMU.class, "imu");
            // Adjust the orientation parameters to match your robot
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, //UP
                    RevHubOrientationOnRobot.UsbFacingDirection.UP));   //FORWARD
            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
            imu.initialize(parameters);

            waitForStart();

            if (isStopRequested()) return;

            while (opModeIsActive()) {
                double y = -gamepad1.left_stick_y;  //-gamepad1.left_stick_y
                double x = gamepad1.left_stick_x;   //gamepad1.left_stick_x
                double rx = (0.8 * gamepad1.right_stick_x);

                // This button choice was made so that it is hard to hit on accident,
                // it can be freely changed based on preference.
                // The equivalent button is start on Xbox-style controllers.
                if (gamepad1.y) {
                    imu.resetYaw();
                }

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX = rotX * 1.2;  // Counteract imperfect strafing

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;


                //----------------------------Drive-Speed----------------------------\\

                /*double P = ((gamepad1.left_trigger) + 0.5);

                lF.setPower(frontLeftPower * P);
                lB.setPower(backLeftPower * P);
                rF.setPower(frontRightPower * P);
                rB.setPower(backRightPower * P);*/

                lF.setPower(0.8 * frontLeftPower);
                lB.setPower(0.8 * backLeftPower);
                rF.setPower(0.8 * frontRightPower);
                rB.setPower(0.8 * backRightPower);

                if (gamepad1.left_bumper) {//speed for going across field
                    lF.setPower(frontLeftPower);
                    lB.setPower(backLeftPower);
                    rF.setPower(frontRightPower);
                    rB.setPower(backRightPower);
                }
                if (gamepad1.right_bumper) {//slow for corrections
                    lF.setPower(0.1 * frontLeftPower);
                    lB.setPower(0.1 * backLeftPower);
                    rF.setPower(0.1 * frontRightPower);
                    rB.setPower(0.1 * backRightPower);
                }

                //----------------------------pivot----------------------------\\

                if (gamepad2.dpad_right) {pivot.setPosition(0.14); }

            /*boolean armButton = gamepad2.a;
            if (armButton != oldArmButton) {
                telemetry.addData("button presss confirmed", 0);
                oldArmButton = armButton;
                if (!armButton) {
                    clawState = !clawState;
                    if (clawState) {
                        pivot.setPosition(0.75);
                        telemetry.addData("srm up",1); }
                    else {pivot.setPosition(0.35);
                        telemetry.addData("srm dn",0); } } }

            telemetry.update();*/

                //----------------------------claw----------------------------\\

                if (gamepad2.left_bumper) {
                    clawL.setPosition(0.33); //close
                    ; }
                else {clawL.setPosition(.45);} //open

                if (gamepad2.right_bumper) {
                    //
                    clawR.setPosition(0.37); } //close
                else {; //
                    clawR.setPosition(.25); } //open
//            if (gamepad2.right_bumper || gamepad2.left_bumper) {
//                clawL.setPosition(0.33); //close
//                clawR.setPosition(0.37); }
//            else {clawL.setPosition(.45); //open
//                clawR.setPosition(.25); }

                //----------------------------hanger----------------------------\\



                //----------------------------droneServo----------------------------\\

                if (gamepad1.a) {
                    droneServo.setPosition(1); }
                else {droneServo.setPosition(0.5); }


                //----------------------------lift/gear----------------------------\\

                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //lift.setPower(1);
                gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //gear.setPower(0.5);

            /*if (lift.getCurrentPosition() < 50 || lift.getTargetPosition() < 50) {
                lift.setTargetPosition(50); }
            if (lift.getCurrentPosition() > 3500 || lift.getTargetPosition() > 3500) {
                lift.setTargetPosition(3500); }*/
                if(gamepad2.left_stick_button){
                    raegPosition(590, 1);//900
                    pivot.setPosition(0.8);//0.23
                    tfilPosition(2050, 0.8);}


                if (gamepad2.dpad_up) {
                    raeg(1); }
                if (gamepad2.dpad_down) {
                    raeg(-1); }
                if (gamepad2.left_trigger > .1) {
                    tfil(-1);
                    telemetry.addData("lift", lift.getCurrentPosition());}
                if (gamepad2.right_trigger > .1) {
                    tfil(1);
                    telemetry.addData("lift", lift.getCurrentPosition());}

                if (gamepad2.dpad_left) {
                    fasttfil(1); }


                if (gamepad1.x) {gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); }


                /**lift encoder should never be above 2800
                 * DO NOT GO FROM HIGH TO RESET**/
                //----------------PRESETS----------------//

                //---------------High---------------\\
                if (gamepad2.x) {
                    pivot.setPosition(0.185);//0.14
                    tfilPosition(2600, 1);//Gear value higher = turn back more & Gear Value lower = turn back less OPPOSITE FOR CLAW PIVOT
                    raegPosition(835, 1); }//845
                //---------------Mid---------------\\
                if (gamepad2.y) {
                    raegPosition(830, 1);//900
                    pivot.setPosition(0.18);//0.23
                    tfilPosition(1900, 0.4); }//1450
                //---------------Low---------------\\
                if (gamepad2.b) {
                    raegPosition(870, 1);//900..
                    pivot.setPosition(0.21);//0.23
                    tfilPosition(1275, 0.4); }//1450
                //---------------Reset---------------\\
                if (gamepad2.a) {
                    tfilPosition(50, 1);
                    raegPosition(35, 0.2);//25
                    pivot.setPosition(0.8); }

                //----------Driver1-Reset----------\\
                if (gamepad1.b) {
                    tfilPosition(50, 1);
                    raegPosition(175, 0.2);
                    pivot.setPosition(0.10); }

                //---------------Hang---------------\\
                if (gamepad2.dpad_right) {
                    raegPosition(625, 0.4);
                    tfilPosition(2000, .7);
                    pivot.setPosition(0.8); }

            /*
            if (gamepad2.x && !gamepad2.y) {
                Tilt(.003); }
             else if (gamepad2.y && !gamepad2.x) {
                Tilt(-.003); }
            else if (gamepad2.x && gamepad2.y) {
                Tilt(0); }
            else {pivot.setPosition(.8 - (gear.getCurrentPosition() * 0.00048)); }
            */

                //pivot.setPosition(0.5 - (gear.getCurrentPosition() * 0.00025324));

                //----------------------------Telemetry----------------------------\\

            /*telemetry.addData("hanger encoder", hanger.getCurrentPosition());
            telemetry.addData("hanger target", lift.getTargetPosition());
            telemetry.addData("lift encoder", lift.getCurrentPosition());
            telemetry.addData("lift target", lift.getTargetPosition());
            telemetry.addData("gear encoder", gear.getCurrentPosition());
            telemetry.addData("gear target", gear.getTargetPosition());
            telemetry.addData("pivot position", pivot.getPosition());
            telemetry.addData("clawL position", clawL.getPosition());
            telemetry.addData("clawR position", clawR.getPosition());
            telemetry.addData("droneServo position", droneServo.getPosition());*/
                telemetry.update();
            }
        }
    }

    public void raegPosition (int h, double H) {gear.setTargetPosition(h); gear.setPower(H); }
    public void tfilPosition (int e, double E) {lift.setTargetPosition(e); lift.setPower(E); }

    public void raeg (int s) {
        gear.setPower(0.333);
        gear.setTargetPosition(gear.getCurrentPosition() + 50 * s);
    }
    public void tfil (int s) {
        lift.setPower(0.95);
        lift.setTargetPosition(lift.getCurrentPosition() + 100 * s);//was 250
        /*if (lift.getTargetPosition() < 50) {
            lift.setTargetPosition(50); }
        if (lift.getTargetPosition() > 3000) {
            lift.setTargetPosition(3000); }*/
    }
    public void fasttfil (int s) {
        lift.setPower(1);
        lift.setTargetPosition(lift.getCurrentPosition() + 250 * s);
        if (lift.getTargetPosition() > 2500) {
            lift.setTargetPosition(2500); }
    }
    //public void Tilt (double p) {pivot.setPosition(pivot.getPosition() + p); }
}
