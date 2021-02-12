package org.firstinspires.ftc.teamcode.Robots;

import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot.EncoderSubsystem;

public class EncoderTestRobot extends Robot {


    public EncoderSubsystem EncoderTestRobot;


    public EncoderTestRobot(OpMode opMode) {
        super(opMode);

        //EncoderTestRobot = new EncoderSubsystem(this, "bottomRight");
        //skystoneDetector = new opencvSkystoneDetector();
    }
}