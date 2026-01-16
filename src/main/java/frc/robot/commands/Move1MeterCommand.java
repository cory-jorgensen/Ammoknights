// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class Move1MeterCommand extends Command {
  private final DriveTrain m_subsystem;
  private double m_sourceDistance;

  /** Creates a new Move1MeterCommand. 
   * Implicitly uses the FrontRight swerve as the distance calculator
  */
  public Move1MeterCommand(DriveTrain dorsalFin) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dorsalFin);
    m_subsystem = dorsalFin;
  }

  // Called when the command is initially scheduled. 
  @Override
  public void initialize() {
    this.m_sourceDistance = m_subsystem.getSwerveDriveLocations()[1]; // 1 Meter Forward
    System.out.print("Target Distance: ");
    System.out.println(this.m_sourceDistance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.drive(0, -1, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    final double frontLeftLocation = m_subsystem.getSwerveDriveLocations()[1];
    System.out.print("Current Distance: ");
    System.out.println(frontLeftLocation);
    return Math.abs(m_sourceDistance - frontLeftLocation) > 1;
  }
}
