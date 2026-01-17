// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.TimeUnit;

//import java.util.Optional;

//import org.photonvision.EstimatedRobotPose;
//import org.photonvision.PhotonCamera;
//import org.photonvision.PhotonPoseEstimator;
//import org.photonvision.PhotonPoseEstimator.PoseStrategy;

//import edu.wpi.first.apriltag.AprilTagFieldLayout;
//import edu.wpi.first.apriltag.AprilTagFields;
//import frc.robot.wrappers.PositionMemory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.*;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefaultWheelCommand;
import frc.robot.commands.WristPositioning;
import frc.robot.commands.ElevatorPositioning;

//import frc.robot.commands.DefaultFilterFeederCommand;
//import frc.robot.commands.FollowAprilTagCommand;
import frc.robot.commands.GoToSpecifiedPosition;
import frc.robot.commands.Move1MeterCommand;
import frc.robot.commands.TuneSwerveAutonomousCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wheel;
import frc.robot.subsystems.Wrist;
//import frc.robot.subsystems.FilterFeeder;    
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer {
  private final CommandXboxController xboxController = new CommandXboxController(0);

  // define SUBSYSTEMS!!!
  private final DriveTrain driveTrain;
  private final Elevator elevator;
  private final Wheel wheel;
  private final Wrist wrist;

  // private final PositionMemory m_elevatorPositionMemory = new
  // PositionMemory(0,3);

  // private final FilterFeeder m_filterFeeder;

  // DEFINE default COMMAND?
  public final DefaultDriveCommand defaultDriveCommand;
  public final Move1MeterCommand move1MeterCommand;

  public final WristPositioning wristPositioning;

  // public final FollowAprilTagCommand followAprilTagCommand;
  public final GoToSpecifiedPosition goToSpecifiedPosition;
  // public final DefaultFilterFeederCommand defaultFilterFeederCommand;

  public final TuneSwerveAutonomousCommand tuneSwerveAutonomousCommand;
  private final SysIdRoutine sysidRoutine;

  // private final PhotonCamera m_camera = new PhotonCamera("Camera_Module_v1");
  // AprilTagFieldLayout aprilTagFieldLayout =
  // AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  Transform3d robotToCam = new Transform3d(
      new Translation3d(0.3302, 0.0, 0.2),
      new Rotation3d(0, 0, 0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
                                // from center.
  // Construct PhotonPoseEstimator
  // PhotonPoseEstimator photonPoseEstimator = new
  // PhotonPoseEstimator(aprilTagFieldLayout,
  // PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCam);

  private static Field2d fieldPose = new Field2d();

  public RobotContainer(Robot robot) {
    driveTrain = new DriveTrain(robot);
    elevator = new Elevator();
    wheel = new Wheel();
    wrist = new Wrist();

    defaultDriveCommand = new DefaultDriveCommand(driveTrain, xboxController);
    move1MeterCommand = new Move1MeterCommand(driveTrain);
    wristPositioning = new WristPositioning(wrist, 0);
    // followAprilTagCommand = new FollowAprilTagCommand(m_dorsalFin, m_camera,
    // robot);
    goToSpecifiedPosition = new GoToSpecifiedPosition(driveTrain, robot);

    // defaultFilterFeederCommand = new DefaultFilterFeederCommand(m_filterFeeder);

    wrist.setDefaultCommand(wristPositioning);
    driveTrain.setDefaultCommand(defaultDriveCommand);
    wheel.setDefaultCommand(new DefaultWheelCommand(wheel, xboxController));
    // elevator.setDefaultCommand(defaultElevatorCommand);
    // m_filterFeeder.setDefaultCommand(defaultFilterFeederCommand);

    tuneSwerveAutonomousCommand = new TuneSwerveAutonomousCommand(driveTrain);
    sysidRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(BaseUnits.VoltageUnit.of(0.1).per(BaseUnits.TimeUnit), BaseUnits.VoltageUnit.of(1.6),
            BaseUnits.TimeUnit.of(10)),
        new SysIdRoutine.Mechanism(driveTrain::sysIdVoltageDrive, driveTrain::driveLogs, driveTrain));
    configureBindings();
  }

  private void configureBindings() {
    xboxController.a().onTrue(new ElevatorPositioning(elevator, kConstants.kElevatorScoreL1Position));
    xboxController.b().onTrue(new ElevatorPositioning(elevator, kConstants.kElevatorScoreL2Position));
    xboxController.x().onTrue(new ElevatorPositioning(elevator, kConstants.kElevatorScoreL3Position));
    xboxController.y().onTrue(new ElevatorPositioning(elevator, kConstants.kElevatorScoreL4Position));

    xboxController.x().onTrue(new WristPositioning(wrist, kConstants.kWristPos3/20));
    xboxController.y().onTrue(new WristPositioning(wrist, kConstants.kWristPos2/20));
    xboxController.b().onTrue(new WristPositioning(wrist, kConstants.kWristPos1/20));
    xboxController.a().onTrue(new WristPositioning(wrist, kConstants.kWristPos0/20));

    xboxController.povDown().onTrue(new WristPositioning(wrist, kConstants.kWristPos3/20));
    xboxController.povLeft().onTrue(new WristPositioning(wrist, kConstants.kWristPos2/20));
    xboxController.povUp().onTrue(new WristPositioning(wrist, kConstants.kWristPos1/20));
    xboxController.back().onTrue(new WristPositioning(wrist, kConstants.kWristPos0/20));

    // if (kConstants.kEnableFeedforwardTuning) {
    // xboxController.a().whileTrue(this.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // xboxController.b().whileTrue(this.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // xboxController.x().whileTrue(this.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // xboxController.y().whileTrue(this.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // }
    // if (kConstants.kEnable1MeterTuning) {
    // xboxController.a().whileTrue(this.move1MeterCommand);
    // }
    // if (kConstants.kEnableFollowApriltag) {
    // // m_driveController.a().whileTrue(this.followAprilTagCommand);
    // }
    // if (kConstants.kEnableGoToSpecifiedPosition) {
    // xboxController.a().whileTrue(this.goToSpecifiedPosition);
    // }
  }

  public Command getAutonomousCommand() {
    return move1MeterCommand;
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysidRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysidRoutine.dynamic(direction);
  }

  // public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
  // photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
  // return Optional.empty();
  // //photonPoseEstimator.update(m_camera.getAllUnreadResults().get(0));
  // }

  // public Pose2d getGlobalPose(){
  // EstimatedRobotPose estimatedRobotPose = getEstimatedGlobalPose().get();
  // return estimatedRobotPose.estimatedPose.toPose2d();
  // }

  public void updateOdometry() {
    driveTrain.updateOdometry();
  }

  public Pose2d getFieldPose() {
    return driveTrain.getFieldPose();
  }

  public Field2d updateFieldPose() {
    fieldPose.setRobotPose(getFieldPose());
    return fieldPose;
  }

  public void putAllSmartDashboardData() {
    // TODO
  }
}
