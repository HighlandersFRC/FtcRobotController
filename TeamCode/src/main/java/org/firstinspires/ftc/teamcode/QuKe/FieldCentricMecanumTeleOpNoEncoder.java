package org.firstinspires.ftc.teamcode.QuKe;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp

public abstract class FieldCentricMecanumTeleOpNoEncoder extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {


        PID1 PID = new PID1();
        // Declare our motors
        DcMotor Left_Front = hardwareMap.dcMotor.get("Left_Front");
        DcMotor Left_Back = hardwareMap.dcMotor.get("Left_Back");
        DcMotor Right_Front = hardwareMap.dcMotor.get("Right_Front");
        DcMotor Right_Back = hardwareMap.dcMotor.get("Right_Back");

        waitForStart();
        while (opModeIsActive()) {

            IMU imu = hardwareMap.get(IMU.class, "imu");
            // Adjust the orientation parameters to match your robot
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP));
            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
            imu.initialize(parameters);
            // Find a motor in the hardware map named "Arm Motor"

            double CPR = 288;
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double botHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double botHeadingRadian = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            if (botHeadingRadian != 0) {
                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(botHeadingRadian) - y * Math.sin(botHeadingRadian);// Changed to positive due to things(change back when need)
                double rotY = x * Math.sin(botHeadingRadian) + y * Math.cos(botHeadingRadian);//Changed to positive due to things(change back when need)

                rotX = rotX * 1.1;  // Counteract imperfect strafing
                // Denominator is the largest motor power (absolute value) or 1rmn
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;


                Left_Front.setPower(-frontLeftPower);
                Left_Back.setPower(-backLeftPower);
                Right_Front.setPower(frontRightPower);
                Right_Back.setPower(backRightPower);
                telemetry.addData("y", y);
                telemetry.addData("x", x);
                telemetry.addData("rx", rx);
                telemetry.addData("rotY", rotY);
                telemetry.addData("rotX", rotX);
                telemetry.addData("parameters", parameters);
                telemetry.addData("IMU", imu);

                telemetry.addData("denominator", denominator);
                telemetry.addData("botHeading", botHeading);
                telemetry.addData("botHeadingRadian", botHeadingRadian);
                telemetry.addData("frontLeftPower", frontLeftPower);
                telemetry.addData("backLeftPower", backLeftPower);
                telemetry.addData("frontRightPower", frontRightPower);
                telemetry.addData("backRightPower", backRightPower);
                telemetry.addData("time", time);


            }
        }
    }
}