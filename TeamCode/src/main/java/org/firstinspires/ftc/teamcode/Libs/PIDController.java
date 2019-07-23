package org.firstinspires.ftc.teamcode.Libs;

//import com.qua l comm.robot core.util.ElapsedTime;

/**
 * Created by Zachary Collins on 2/10/19.
 * Instagram: @hilariously_random
 *
 * A PID controller adapted from python to java
 */

public class PIDController {
    private double kP;
    private double kI;
    private double kD;
    private double dt;
    private double error;
    private double lastError;
    private double integral = 0;
    private boolean log;
    private double previousTime;
    private DataLogger Dl = new DataLogger("PID_LOG");

    public PIDController(double kP, double kI, double kD, boolean log){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.log = log;
    }

    public double getOutput(double current, double setPoint){
        double currentTime = (double)System.currentTimeMillis();
        this.dt = (currentTime - this.previousTime);

        this.error = setPoint - current;

        double proportional = this.kP * error;
        this.integral += this.kI * (error + this.lastError) * dt;
        double derivative = this.kD * (error - this.lastError) / dt;
        double output = proportional + integral + derivative;

        this.lastError = error;
        this.previousTime = currentTime;
        if(this.log){
            Dl.addField(current);
            Dl.addField(setPoint);
            Dl.addField(error);
        }
        return output;
    }
    public double getError(){
        return this.error;
    }
    public boolean resetIntegral() { this.kI = 0; return true; }
    public boolean closeLog(){
        Dl.closeDataLogger();
        return true;
    }
}