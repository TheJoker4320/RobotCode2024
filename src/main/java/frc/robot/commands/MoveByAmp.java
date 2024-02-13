package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ClawConstants.CURRENT_PID;

public class MoveClawByAmp extends CommandBase {

    private final Claw claw;
    private final DoubleSupplier currentSupplier;


    public MoveClawByAmp(DoubleSupplier currentSupplier) {
        claw = Claw.getInstance();
        this.currentSupplier = currentSupplier;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        claw.setPidController(CURRENT_PID);
    }

    @Override
    public void execute() {
        claw.setCurrent(currentSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        claw.setSpeed(0);
    }
}
