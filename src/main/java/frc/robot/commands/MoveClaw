package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ClawConstants.CURRENT_PID;
import static frc.robot.Constants.ClawConstants.OPEN_SPEED;

public class MoveClaw extends CommandBase {

    private final Claw claw;
    public MoveClaw() {
        claw = Claw.getInstance();
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        claw.setSpeed(OPEN_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        claw.stop();
    }
}
