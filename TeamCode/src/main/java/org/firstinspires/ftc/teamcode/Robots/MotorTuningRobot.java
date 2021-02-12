package org.firstinspires.ftc.teamcode.Robots;
import com.SCHSRobotics.HAL9001.system.robot.Robot;


import com.SCHSRobotics.HAL9001.system.robot.subsystems.drivetrain.MecanumDrive;
import com.SCHSRobotics.HAL9001.util.control.Button;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorTuningRobot extends Robot {

    public MecanumDrive mDrive;
    public MotorTuningRobot(OpMode opMode) {
        super(opMode);
        mDrive = new MechanumDrive(this, new MechanumDrive.Params("topLeft", "topRight", "bottomLeft", "bottomRight")
                .setDriveStick(new Button(1, Button.VectorInputs.left_stick))
                .setRevHubsInverted(true)
                .setTurnStick(new Button(1, Button.DoubleInputs.right_stick_x))
                .setConstantSpeedModifier(1) .setSpeedModeMultiplier(.5)
                .setSpeedModeButton(new Button(1, Button.BooleanInputs.a))
                .setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        putSubSystem(":)", mDrive);
    }
}
