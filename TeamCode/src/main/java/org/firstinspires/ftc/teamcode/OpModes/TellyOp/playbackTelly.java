package org.firstinspires.ftc.teamcode.OpModes.TellyOp;


import com.SCHSRobotics.HAL9001.system.config.StandAlone;
import com.SCHSRobotics.HAL9001.system.robot.BaseTeleop;
import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.SCHSRobotics.HAL9001.util.control.CustomizableGamepad;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.io.*;

import org.firstinspires.ftc.teamcode.Robots.MainRobot;

@StandAlone
@TeleOp(name = "Playback Telly")
public class playbackTelly extends BaseTeleop {
    //file writer
    public PrintWriter pw = null;
    // get button inputs
    private CustomizableGamepad inputs;

    MainRobot robot;
    @Override
    protected Robot buildRobot() {
        robot = new MainRobot(this);
        return robot;
    }

    @Override
    protected void onStart() {
        try {
            pw = new PrintWriter(new BufferedWriter(new FileWriter("playbackTellyOutputs.txt")));
        } catch (IOException e) {
            e.printStackTrace();
            telemetry.addLine("Error opening file, error was: " + e.toString());
        }
        

    }

    @Override
    protected void onUpdate() {
        super.onUpdate();
        Pose2d curpos = robot.mDrive.getPoseEstimate();
  // finish this line lol THE ZHANGGGGG
        pw.println(robot.odometry.returnXCoordinate());
        pw.println(robot.odometry.returnYCoordinate());

        pw.println(curpos.getX());
        pw.println(curpos.getY());
        pw.println(curpos.getHeading());

        // May be needed, updates the x,y coordinates
        // pw.println(globalCoordinatePositionUpdate); //

        //pw.println(robot.odometry.); // Mark the servo
        pw.println(robot.odometry.returnOrientation());
        pw.close();
        robot.mDrive.updateLocalizer();
    }
}