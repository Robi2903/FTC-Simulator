package InternalFiles;

public class PID_Controller {
    private float kp, ki, kd;
    private float integral, previousError;

    public PID_Controller(float kp, float ki, float kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.integral = 0;
        this.previousError = 0;
    }

    public double calculate(float error) {
        integral += error;
        float derivative = error - previousError;
        previousError = error;
        return kp * error + ki * integral + kd * derivative;
    }

    // Other existing methods...
}