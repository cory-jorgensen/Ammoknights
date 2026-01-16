// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.kConstants;

public class Wrist extends SubsystemBase {

    private final SparkMax wristMotor;

    // wrist is guaranteed to be starting at the top
    // wrist is in the range of 0--0.35                ????????????????
    private final ProfiledPIDController wristPIDController = new ProfiledPIDController(
            kConstants.kWristKP,
            kConstants.kWristKI,
            kConstants.kWristKD,
            new TrapezoidProfile.Constraints(
                    kConstants.kWristMaxVelocity,
                    kConstants.kWristMaxAcceleration));

    /** Creates a new Elevator. */
    public Wrist() {
        wristMotor = new SparkMax(kConstants.kWristCanId, MotorType.kBrushless);

        wristPIDController.setTolerance(0.0001, 0.0001);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Position: ", this.getWristPos());
        SmartDashboard.putNumber("Wrist target: ", this.wristPIDController.getGoal().position);
    }

    public double getWristPos() {
        return (wristMotor.getEncoder().getPosition()) * kConstants.kElevatorRatio
                * kConstants.kElevatorRotationsToInches;
    }

    public void reset() {
        wristPIDController.reset(this.getWristPos());
    }

    public void setPIDTarget(double position) {
        wristPIDController.setGoal(position);
    }

    public boolean atGoal() {
        return wristPIDController.atGoal();
    }

    public void setMotorPower(double power) {
        wristMotor.setVoltage(power);
    }

    public void updateMotorState() {
        double motorPower = wristPIDController.calculate(getWristPos());
        double wristTarget = this.wristPIDController.getGoal().position;

        if (wristTarget > 0.0 || wristTarget < kConstants.kWristPos3) {
            motorPower = 0.0;
        }

        setMotorPower(motorPower);
    }
}