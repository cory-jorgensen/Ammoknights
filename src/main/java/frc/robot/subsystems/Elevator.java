// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.SparkBase.PersistMode;
//import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.kConstants;

public class Elevator extends SubsystemBase {

    private final SparkMax m_elevatorMotor;
    private final SparkMax m_elevatorMotor2;

    // elevator is guaranteed to be starting at the bottom
    // elevator is in the range of 0-3.35
    private final ProfiledPIDController elevatorPIDController = new ProfiledPIDController(
            kConstants.kElevatorKP,
            kConstants.kElevatorKI,
            kConstants.kElevatorKD,
            new TrapezoidProfile.Constraints(
                    kConstants.kElevatorMaxVelocity,
                    kConstants.kElevatorMaxAcceleration));

    private boolean m_configuredForUp = false;

    /** Creates a new Elevator. */
    public Elevator() {
        m_elevatorMotor = new SparkMax(kConstants.kElevatorMotor, MotorType.kBrushless);
        // m_elevatorMotor.configure(kConstants.kNeoLowStallCurrentConfig,
        // ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_elevatorMotor2 = new SparkMax(kConstants.kElevatorMotor2, MotorType.kBrushless);
        // m_elevatorMotor2.configure(kConstants.kNeoLowStallCurrentConfig,
        // ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        elevatorPIDController.setTolerance(0.01, 0.01);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position: ", this.getElevatorPos());
        SmartDashboard.putNumber("Elevator target: ", this.elevatorPIDController.getGoal().position);
    }

    public double getElevatorPos() {
        return (m_elevatorMotor.getEncoder().getPosition()) * kConstants.kElevatorRatio
                * kConstants.kElevatorRotationsToInches;
    }

    public void reset() {
        elevatorPIDController.reset(this.getElevatorPos());
    }

    public void setPIDTarget(double position) {
        elevatorPIDController.setGoal(position);
    }

    public boolean atGoal() {
        return elevatorPIDController.atGoal();
    }

    public void setMotorPower(double power) {
        m_elevatorMotor.setVoltage(power);
        m_elevatorMotor2.setVoltage(-power);
    }

    public void updateMotorState() {
        double motorPower = elevatorPIDController.calculate(getElevatorPos());
        double elevatorTarget = this.elevatorPIDController.getGoal().position;

        if (elevatorTarget < 0.0 || elevatorTarget > 3.35) {
            motorPower = 0.0;
        }

        if (motorPower > 0 && !m_configuredForUp) {
            // m_elevatorMotor.configure(kConstants.kNeoHighStallCurrentConfig,
            // ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
            // m_elevatorMotor2.configure(kConstants.kNeoHighStallCurrentConfig,
            // ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

            m_configuredForUp = true;
        }
        if (motorPower < 0 && m_configuredForUp) {
            // m_elevatorMotor.configure(kConstants.kNeoLowStallCurrentConfig,
            // ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
            // m_elevatorMotor2.configure(kConstants.kNeoLowStallCurrentConfig,
            // ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

            m_configuredForUp = false;
        }

        setMotorPower(motorPower);
    }
}