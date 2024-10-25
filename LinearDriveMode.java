package org.firstinspires.ftc.teamcode.drive.opmodetele;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.geometry.Pose2d;


import org.firstinspires.ftc.teamcode.drive.robot.Robot;

@TeleOp(name="MecanumDriveMode", group="Linear OpMode")

public class LinearDriveMode extends LinearOpMode {
    private Robot robot = null;
    int direction = 1; 
    double servoPosSlides = 0.5;
    double servoPosGrippy = 0;

    // reguleaza deplasarea crane-ului
    public double calculateThrottle(float x) {
        int sign = -1;
        if (x > 0) sign = 1;
        return sign * 3 * abs(x);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // log
        telemetry.addData(">", "Initializing...");
        telemetry.update();

        // initializeaza robotul si se asigura ca e in opmode
        robot = new Robot(hardwareMap);
        while (robot.isInitialize() && opModeIsActive()) {
            idle();
        }

        // tot logging
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData(">", "Initialized");
        telemetry.update();



        // asteapta startul si se opreste daca trebuie
        waitForStart();
        if (isStopRequested()) return;


        while (opModeIsActive()) { // cat timp in opmode
            
            // daca left bumper atunci se extinde(?) crane-ul,
            // daca right bumper se strange

            if (gamepad2.left_bumper) {
                robot.crane.slidesDirection = 1;
                robot.crane.setSlides(5);
                if(robot.crane.slideEncoderLastPosition > robot.crane.slideEncoder.getVoltage()){ // se asigura ca nu a trecut de pozitia
                    robot.crane.slideExtension -= 3.3;
                }
            } else if (gamepad2.right_bumper) {
                robot.crane.slidesDirection = -1;
                robot.crane.setSlides(5);
                if(robot.crane.slideEncoderLastPosition < robot.crane.slideEncoder.getVoltage()){
                    robot.crane.slideExtension += 3.3;
                }
            } else {
               robot.crane.setSlides(0);
               // nu se misca
            }
            robot.crane.slideEncoderLastPosition = robot.crane.slideEncoder.getVoltage(); // seteaza ultima pozitie


            // daca left trigger, seteaza targetul mai departe(?) de robot, daca dreapta, mai aproape
            if(gamepad2.left_trigger > 0.1){
                robot.crane.craneTarget -= (int) calculateThrottle(gamepad2.left_trigger);
            }
            else if(gamepad2.right_trigger > 0.1){
                robot.crane.craneTarget += (int) calculateThrottle(gamepad2.right_trigger);
            }
            robot.crane.motorCrane1.setPower(robot.crane.cranePower(robot.crane.craneTarget)); // seteaza puterea
            robot.crane.motorCrane2.setPower(robot.crane.cranePower(robot.crane.craneTarget)); // seteaza puterea

            // dac apasa pe a, starnge(?), pe b desface, altfel nmc
            if (gamepad2.a) {
                robot.crane.gripperDirection = 1;
                robot.crane.setGripper(1);
            }
            else if (gamepad2.b) {
                robot.crane.gripperDirection = -1;
                robot.crane.setGripper(1);
            }
            else robot.crane.setGripper(0);

            // misca robotul efectiv in functie de inputul de la joystick uri
            robot.drive.setWeightedDrivePower(new Pose2d((-gamepad1.left_stick_y),(-gamepad1.left_stick_x),(-gamepad1.right_stick_x)));


            // tot logging
            telemetry.addData("crane target: ", robot.crane.craneTarget);
                telemetry.addData("right trigger: ", gamepad2.right_trigger);
                telemetry.addData("encoder value: ", robot.crane.slideEncoder.getVoltage());
                telemetry.addData("last position ", robot.crane.slideEncoderLastPosition);
                telemetry.addData("slide extension ", robot.crane.slideExtension);
                telemetry.addData("sensor touch: ", robot.crane.slideSensor.isPressed());
//                telemetry.addData("CRANE TICKS LEFT: ", robot.crane.motorCraneLeft.getCurrentPosition());
//                telemetry.addData("CRANE TICKS RIGHT: ", robot.crane.motorCraneRight.getCurrentPosition());
//                telemetry.addData("DIRECTION: ", direction);
//                telemetry.addData("SERVO GRIPPER: ", robot.crane.servoGrippy1.getPosition());
                telemetry.update();
            }

        }

    }



