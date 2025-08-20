package frc.robot.commands.auton;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class RepeatUntilAfterIteration extends Command {
    private final Supplier<Command> detourFactory;
    private final Supplier<Boolean> condition; // returns true when we should stop
    private Command currentDetour = null;
    private boolean finished = false;

    /**
     * @param detourFactory Produces a fresh Command to run for one detour iteration.
     *                      (recommended to use a Supplier so each iteration is a fresh command instance)
     * @param condition     Returns true when we should stop repeating (checked only after each detour finishes)
     */
    public RepeatUntilAfterIteration(Supplier<Command> detourFactory, Supplier<Boolean> condition) {
        this.detourFactory = detourFactory;
        this.condition = condition;
    }

    @Override
    public void initialize() {
        // If condition already true, finish immediately
        if (condition.get()) {
            finished = true;
            return;
        }
        // Otherwise schedule first detour
        scheduleNextDetour();
    }

    private void scheduleNextDetour() {
        // create a fresh detour and schedule it
        currentDetour = detourFactory.get();
        if (currentDetour != null) {
            CommandScheduler.getInstance().schedule(currentDetour);
        } else {
            // defensive: if factory returns null, stop
            finished = true;
        }
    }

    @Override
    public void execute() {
        if (finished) {
            return;
        }

        // If detour was scheduled previously and is now finished (not scheduled),
        // check the condition and either finish or schedule another detour.
        if (currentDetour == null || !currentDetour.isScheduled()) {
            // Detour completed (or never scheduled) — check sensor now
            if (condition.get()) {
                finished = true;
            } else {
                scheduleNextDetour();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        // If we leave early, cancel any running detour
        if (currentDetour != null && currentDetour.isScheduled()) {
            currentDetour.cancel();
        }
    }
}
