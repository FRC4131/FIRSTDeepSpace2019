package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class DisplacementAHRS extends AHRS {
    private enum PIDSourceType {
        DisplacementX,
        DisplacementY,
        DisplacementVector,
    }

    private PIDSourceType pidSourceType = PIDSourceType.DisplacementVector;

    public DisplacementAHRS(SPI.Port spi_port_id) {
        super(spi_port_id);
    }

    public PIDSourceType getPidSourceType() {
        return pidSourceType;
    }

    public void setPidSourceType(PIDSourceType pidSourceType) {
        this.pidSourceType = pidSourceType;
    }

    @Override
    public double pidGet() {
        switch (pidSourceType) {
            case DisplacementVector:
                return Math.sqrt(
                        Math.pow(this.getDisplacementX(), 2) +
                                Math.pow(this.getDisplacementY(), 2) +
                                Math.pow(this.getDisplacementZ(), 2)
                );
            case DisplacementX:
                return this.getDisplacementX();
            case DisplacementY:
                return this.getDisplacementY();
        }
        return 0;
    }
}
