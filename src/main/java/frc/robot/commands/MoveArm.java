package frc.robot.commands;

import frc.robot.subsystems.Arm;

import frc.robot.Constants.ArmConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        if (!((arm.getPosition() > ArmConstants.MAX_DEGREES && !isReversed && arm.getPosition() < 350)
                || (arm.getPosition() < ArmConstants.MIN_DEGREES && isReversed)))
            ;
        arm.setSpeed(ArmConstants.SPEED * reversed);
        SmartDashboard.putNumber("speed", ArmConstants.SPEED * reversed);
        SmartDashboard.putBoolean("isReversed", isReversed);        
        SmartDashboard.putBoolean("Stopped",
                (arm.getPosition() > ArmConstants.MAX_DEGREES && !isReversed && arm.getPosition() < 350));
        SmartDashboard.putBoolean("Stopped1", (arm.getPosition() < ArmConstants.MIN_DEGREES && isReversed));

    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }

    @Override
    public boolean isFinished() {
        return ((arm.getPosition() > ArmConstants.MAX_DEGREES && !isReversed && arm.getPosition() < 350)
                || (arm.getPosition() < ArmConstants.MIN_DEGREES && isReversed));
    }

}
