package org.firstinspires.ftc.teamcode.commands;

public abstract class Command {
    public boolean commandCompleted = false;
    public abstract void start();
    public abstract void execute();
    public abstract void end();
    public abstract boolean isFinished();
}