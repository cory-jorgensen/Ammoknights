package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Wheel;

public class DefaultWheelCommand extends Command {
    private final Wheel wheel;
    private final CommandXboxController controller;

    public DefaultWheelCommand(Wheel wheel, CommandXboxController xboxController) {
        this.wheel = wheel;
        this.controller = xboxController;

        addRequirements(wheel);
    }

    @Override
    public void execute() {
        double rightTriggerPos = controller.getRightTriggerAxis();
        double leftTriggerPos = -controller.getLeftTriggerAxis();

        double avg = (rightTriggerPos + leftTriggerPos) / 2.0;

        wheel.setSpeed(avg);
    }
}
