package org.firstinspires.ftc.teamcode;

import android.hardware.camera2.CameraDevice;

import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

public class ClosableVuforiaLocalizer extends VuforiaLocalizerImpl {
    boolean closed = false;
    public ClosableVuforiaLocalizer(VuforiaLocalizer.Parameters parameters) {
        super(parameters);
    }
    @Override
    public void close() {
        if (!closed) {
            super.close();
        }
        closed = true;
    }
}
