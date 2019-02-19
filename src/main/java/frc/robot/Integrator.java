package frc.robot;

public class Integrator {
    private double[] last_velocity = new double[2];
    private double[] displacement = new double[2];

    public Integrator() {
        resetDisplacement();
    }

    public void updateDisplacement(double accel_x_g, double accel_y_g, int update_rate_hz, boolean is_moving) {
        if (is_moving) {
            double[] accel_g = new double[2];
            double[] accel_m_s2 = new double[2];
            double[] curr_velocity_m_s = new double[2];
            double sample_time = (1.0d / update_rate_hz);
            accel_g[0] = accel_x_g;
            accel_g[1] = accel_y_g;
            for (int i = 0; i < 2; i++) {
                accel_m_s2[i] = accel_g[i] * 9.80665d;
                curr_velocity_m_s[i] = last_velocity[i] + (accel_m_s2[i] * sample_time);
                displacement[i] += last_velocity[i] + (0.5d * accel_m_s2[i] * sample_time * sample_time);
                last_velocity[i] = curr_velocity_m_s[i];
            }
        } else {
            last_velocity[0] = 0.0f;
            last_velocity[1] = 0.0f;
        }
    }

    public void resetDisplacement() {
        for (int i = 0; i < 2; i++) {
            last_velocity[i] = 0.0f;
            displacement[i] = 0.0f;
        }
    }

    public double getVelocityX() {
        return last_velocity[0];
    }

    public double getVelocityY() {
        return last_velocity[1];
    }

    public float getVelocityZ() {
        return 0;
    }

    public double getDisplacementX() {
        return displacement[0];
    }

    public double getDisplacementY() {
        return displacement[1];
    }

    public float getDisplacementZ() {
        return 0;
    }
}