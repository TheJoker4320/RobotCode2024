package frc.robot.commands;

import frc.robot.subsystems.Arm;

import static frc.robot.Constants.ClawConstants.MAX_DEGREES;
import static frc.robot.Constants.ClawConstants.MIN_DEGREES;
import static frc.robot.Constants.ClawConstants.OPEN_SPEED;
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
        arm.setSpeed(OPEN_SPEED * reversed);
        if ((arm.getPosition() >= MAX_DEGREES - 1 && !isReversed) || (arm.getPosition() <= MIN_DEGREES + 1 && isReversed));
             arm.stop();
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}
