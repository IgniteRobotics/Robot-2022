package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

public class ContinuousConditionalCommand extends ConditionalCommand {
    public ContinuousConditionalCommand(Command onTrue, Command onFalse, BooleanSupplier condition) {
        super(onTrue, onFalse, condition);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
