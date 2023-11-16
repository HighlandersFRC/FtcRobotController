package org.firstinspires.ftc.teamcode.commands;


public class Wait extends Command {
        long time;
    long endTime;
    public Wait(long time){
                this.time = time;
    }
    public void start(){
        endTime = System.currentTimeMillis() + time;
    }
    public void execute(){

    }
    public void end(){
    }

    public boolean isFinished() {
        if (System.currentTimeMillis() >= endTime){
            return true;
        }
        else {
            return false;
        }
    }
}