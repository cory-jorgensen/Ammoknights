package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class kConstants {
    public static final double kDriveGearRatio = 8.14;
    public static final double kWheelDiameter = 4.00000;
    // inches
    public static final double kMaxAngularVelocity = 10 * 40;
    public static final double kMaxAngularAcceleration = 20 * 10;
    public static final double kInchesToMeters = 0.0254;

    public static final double kSwerveDriveKP = 0.44072;
    public static final double kSwerveDriveKI = 0.0;
    public static final double kSwerveDriveKD = 0.0;
    public static final double kSwerveDriveKS = 0.11831;
    public static final double kSwerveDriveKV = 2.4461;
    public static final double kSwerveDriveKA = 0.28161;
    // PID values, modify if vibrating
    public static final double kSwerveTurningKP = 6; // 8
    public static final double kSwerveTurningKI = 0; // 0
    public static final double kSwerveTurningKD = 0.1; // 0
    public static final double kSwerveTurningKS = 0; // 0.2
    public static final double kSwerveTurningKV = 0; // 47.12

    public static final double kDriveTrainWidth = 25.5 * kInchesToMeters;
    public static final double kDriveTrainLength = 24.5 * kInchesToMeters;

    // Swerve motors CAN ID
    public static final SparkBaseConfig kSwerveNominalConfig = new SparkMaxConfig().smartCurrentLimit(150)
            .idleMode(IdleMode.kBrake).inverted(false);

    public static final int kSwerveFrontRightTurnMotor = 6; // 6
    public static final int kSwerveFrontRightDriveMotor = 5; // 5
    public static final int kSwerveFrontRightEncoder = 13;
    public static final double kSwerveFrontRightOffset = 0;

    public static final int kSwerveFrontLeftTurnMotor = 8; // 8
    public static final int kSwerveFrontLeftDriveMotor = 7; // 7
    public static final int kSwerveFrontLeftEncoder = 14;
    public static final double kSwerveFrontLeftOffset = 0;

    public static final int kSwerveBackRightTurnMotor = 4;
    public static final int kSwerveBackRightDriveMotor = 3;
    public static final int kSwerveBackRightEncoder = 15;
    public static final double kSwerveBackRightOffset = 0;

    public static final int kSwerveBackLeftTurnMotor = 2;
    public static final int kSwerveBackLeftDriveMotor = 1;
    public static final int kSwerveBackLeftEncoder = 16;
    public static final double kSwerveBackLeftOffset = 0;

    // Elevator adjust for additional motor (add 10)
    public static final int kElevatorMotor = 9;
    public static final int kElevatorMotor2 = 10;
    public static final double kElevatorRatio = 1.0 / 60.0;

    public static final double kElevatorKP = 15; // TUNE
    public static final double kElevatorKI = 0; // TUNE
    public static final double kElevatorKD = 0; // TUNE

    public static final double kElevatorMaxAcceleration = 80; // TUNE
    public static final double kElevatorMaxVelocity = 120; // TUNE

    public static final double kElevatorScoreL1Position = 0;
    public static final double kElevatorScoreL2Position = 2.15; // TUNE
    public static final double kElevatorScoreL3Position = 2.55; // TUNE
    public static final double kElevatorScoreL4Position = 3.35;

    public static final double kElevatorRotationsToInches = 1; // TUNE

    // wheel
    public static final int kWheelCanId = 12;

    // wrist
    public static final int kWristCanId = 11;
    public static final double kWristKP = 40; // TUNE
    public static final double kWristKI = 8; // TUNE
    public static final double kWristKD = 0.3; // TUNE
    public static final double kWristMaxAcceleration = 0.4; // TUNE
    public static final double kWristMaxVelocity = 0.04; // TUNE
    public static final double kWristPos0 = 0;
    public static final double kWristPos1 = -Math.PI/6.0;
    public static final double kWristPos2 = -Math.PI/3.4;
    public static final double kWristPos3 = -Math.PI/2;

    // Intake
    // TODO Once this assembly is actully added, check the PIDs
    public static final int kIntakeMotor = 20;
    public static final int kIntakeAssemblyMotor = 21;
    public static final int kIndexMotor = 22;
    public static final double kIntakeAssemblyKP = 1;
    public static final double kIntakeAssemblyKI = 0;
    public static final double kIntakeAssemblyKD = 0;
    public static final double kIntakeAssemblyMaxVelocity = 1;
    public static final double kIntakeAssemblyMaxAcceleration = 1;
    public static final double kIntakeAssemblyDownPoint = -1;
    public static final double kIntakeAssemblyUpPoint = 0;
    public static final double kEnableIndex = 4;
    public static final double kEnableIntake = 7;
    public static final SparkBaseConfig kIntakeAssemblyNominalConfig = new SparkMaxConfig().smartCurrentLimit(95)
            .idleMode(IdleMode.kBrake).inverted(false);

    // Manipulator
    public static final double kManipulatorRatio = 1.0 / 35.0;
    public static final int kManipulatorMotor = 23;
    public static final double kManipulatorKP = 1;
    public static final double kManipulatorKI = 0;
    public static final double kManipulatorKD = 0;
    public static final double kManipulatorMaxAcceleration = 1;
    public static final double kManipulatorMaxVelocity = 1;
    public static final double kManipulatorMaxAngle = 180;

    public static final SparkBaseConfig kDefaultNeo550NominalConfig = new SparkMaxConfig().smartCurrentLimit(95)
            .idleMode(IdleMode.kBrake).inverted(false);

    // Feature Flags
    public static final boolean kEnableFeedforwardTuning = false; // A, B, X, Y run feedforward tuning code for the
                                                                  // Sysid tool
    public static final boolean kEnable1MeterTuning = false; // A moves the robot forward 1 meter
    public static final boolean kEnableFollowApriltag = false; // A follows april tag 2 at a distance of 1 meter
    public static final boolean kEnableGoToSpecifiedPosition = false; // A goes to (currently) 0,0 the place where the
                                                                      // robot was restarted
}
/* kConstants.kSwerve */