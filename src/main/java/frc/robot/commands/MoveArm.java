package frc.robot.commands;

import frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.Command;



public class MoveArm extends Command {
    private boolean isReversed;
    private Arm arm;
    public MoveArm(Arm arm, boolean isReversed) {
        this.arm = arm;
        this.isReversed = isReversed;
        addRequirements(arm);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        int reversed = isReversed ? -1 : 1;
        arm.setSpeed(reversed);
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}