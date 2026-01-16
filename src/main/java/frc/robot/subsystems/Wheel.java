package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.kConstants;

public class Wheel extends SubsystemBase {
    private final SparkMax wheelMotor;

    public Wheel() {
        wheelMotor = new SparkMax(kConstants.kWheelCanId, MotorType.kBrushless);
    }

    public void setSpeed(double speed) {
        wheelMotor.set(speed);
    }
}
