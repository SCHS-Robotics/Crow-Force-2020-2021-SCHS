package org.firstinspires.ftc.teamcode.Robots;
import com.SCHSRobotics.HAL9001.system.robot.*;


import com.SCHSRobotics.HAL9001.system.robot.localizer.HolonomicDriveEncoderIMULocalizer;
import com.SCHSRobotics.HAL9001.system.robot.roadrunner_util.RoadrunnerConfig;
import com.SCHSRobotics.HAL9001.system.robot.subsystems.drivetrain.MecanumDrive;
import com.SCHSRobotics.HAL9001.util.control.Button;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorTuningRobot extends Robot {

    public MecanumDrive mDrive;
    public MotorTuningRobot(OpMode opMode) {
        super(opMode);
/*
        mDrive = new MecanumDrive(this, new MecanumDrive.Params("topLeft", "topRight", "bottomLeft", "bottomRight")
                .setDriveStick(new Button(1, Button.VectorInputs.left_stick))
                .setRevHubsInverted(true)
                .setTurnStick(new Button(1, Button.DoubleInputs.right_stick_x))
                .setConstantSpeedModifier(1) .setSpeedModeMultiplier(.5)
                .setSpeedModeButton(new Button(1, Button.BooleanInputs.a))
                .setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        putSubSystem(":)", mDrive);
*/

        mDrive = new MecanumDrive(
                this,
                new RoadrunnerConfig(1.88976, 1, 13.359, 383.6, 435),
                "topLeft",
                "topRight",
                "bottomLeft",
                "bottomRight", false);
        mDrive.setLocalizer(new HolonomicDriveEncoderIMULocalizer(
                this,
                mDrive,
                "imu",
                "topLeft",
                "topRight",
                "bottomLeft",
                "bottomRight"
        ));
        mDrive.setDriveStick(new Button(1, Button.VectorInputs.left_stick));
        mDrive.setTurnStick(new Button(1, Button.DoubleInputs.right_stick_x));
        mDrive.setReverseType(MecanumDrive.ReverseType.LEFT);
        mDrive.setAllMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // putSubSystem(":)", mDrive); Deprecated??
    }
}
