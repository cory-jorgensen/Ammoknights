// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.kConstants;
import frc.robot.subsystems.Elevator;
//import frc.robot.wrappers.Controller;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorPositioning extends Command {
  /** Creates a new DefaultElevatorCommand. */
  private final Elevator elevator;
  private double target;

  public ElevatorPositioning(Elevator elevator, double target) {
    this.elevator = elevator;
    this.target = target;

    elevator.reset();
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setPIDTarget(target);
    elevator.updateMotorState();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // TODO:  This may not work for you.  With OnTrue
    elevator.setMotorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.atGoal();
  }
}