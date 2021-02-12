package org.firstinspires.ftc.teamcode.Robots;


import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.SCHSRobotics.HAL9001.system.robot.subsystems.drivetrain.MecanumDrive;
import com.SCHSRobotics.HAL9001.util.control.Button;
import com.SCHSRobotics.HAL9001.util.control.PIDController;
import com.SCHSRobotics.HAL9001.util.functional_interfaces.BiFunction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.OneTimeUse.TestMecOp;

import static java.lang.Math.PI;

public class TestMec extends Robot {

    public MecanumDrive mDrive;
    public TestMecOp testMec;


    public TestMec(OpMode opMode) {
        super(opMode);
        PIDController dsa = new PIDController(1.3, 0, 0, new BiFunction<Double,Double,Double>() {
            @Override
            public Double apply(Double target, Double current) {
                BiFunction<Double, Double, Double> mod = new BiFunction<Double, Double, Double>() {
                    @Override
                    public Double apply(Double x, Double m) {
                        return (x % m + m) % m;
                    }
                };

                double m = 2 * PI;

                //cw - ccw +
                double cw = mod.apply(mod.apply(current, m) - mod.apply(target, m), m);
                double ccw = -mod.apply(mod.apply(target, m) - mod.apply(current, m), m);

                return Math.abs(ccw) < Math.abs(cw) ? ccw : cw;
            }
        });
        dsa.deadband = PI / 90;

        startGui(new Button(1, Button.BooleanInputs.y));

        mDrive = new MechanumDrive(this, new MechanumDrive.Params("topLeft", "topRight", "bottomLeft", "bottomRight")
                .setDriveStick(new Button(1, Button.VectorInputs.left_stick))
                .setTurnStick(new Button(1, Button.DoubleInputs.right_stick_x))
                .setConstantSpeedModifier(1)
                .setSpeedModeMultiplier(.5)
                .setSpeedModeButton(new Button(1, Button.BooleanInputs.a))
                .setTurnPID(dsa)
                .setImuNumber(2));          //blockIntakeMotor = new IntakeSubSystemMotors(this,"blockIntakeLeft", "blockIntakeRight");

        mDrive.getMotors()[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mDrive.getMotors()[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mDrive.getMotors()[2].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mDrive.getMotors()[3].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        putSubSystem("MechanumDrive", mDrive);

        testMec = new TestMecOp(this);
        putSubSystem("TestMec", testMec);
    }
}