package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.*;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.List;

@Configurable
@TeleOp(name = "Tests", group = "Pedro Pathing")
public class Tests extends SelectableOpMode {

    public Tests() {
        super("Selecciona un test", s -> {
            s.folder("Tests Robot", l -> {
                l.add("PID test - Limelight", PIDTest::new);
                l.add("Pinpoint", PinpointTest::new);
                l.add("Detector de color", ColorDetectorTest::new);
            });
        });
    }
}

class PIDTest extends OpMode {

    private Limelight3A limaluz;
    private Servo Lanzador_I, Lanzador_D;

    private static final int pipelineLeer = 5;
    private static final int pipelineAlinear = 6;
    public int leidoID = -1;

    private static final int preLeerTagObelisco = 0;
    private static final int preAlinear = 1;
    private static final int preListo = 3;

    private int preEstado = preLeerTagObelisco;

    private static final double kP = 0.01;
    private static final double kD = 0.0006;
    private static final double kI = 0.0015;
    private double errorAnterior = 0.0;
    private double integralError = 0.0;
    private static final double integralMax = 250.0;
    private long pidTimer = 0;

    private static final double SERVO_CENTER = 0.5;
    private static final double SERVO_RANGE = 0.25;

    private double correccion = SERVO_CENTER;

    private static final double tolerancia = 0.29;

    private static final long p5Ms = 25;
    private long preStart = 0;
    private long alignStart = 0;

    private void centrarLanzadorServos() {
        Lanzador_I.setPosition(SERVO_CENTER);
        Lanzador_D.setPosition(SERVO_CENTER);
        correccion = SERVO_CENTER;
    }

    private void resetPID() {
        pidTimer = 0;
        errorAnterior = 0.0;
        integralError = 0.0;
    }

    private boolean alinear() {
        LLResult res = limaluz.getLatestResult();

        boolean hayTag = false;
        double tx = 0.0;

        if (res != null && res.isValid() && res.getStaleness() <= 10) {
            List<LLResultTypes.FiducialResult> tags = res.getFiducialResults();
            hayTag = (tags != null && !tags.isEmpty());
            if (hayTag) tx = res.getTx();
        }

        if (!hayTag) {
            centrarLanzadorServos();
            resetPID();
            return false;
        }

        double error = -tx;

        if (Math.abs(error) < tolerancia) {
            centrarLanzadorServos();
            resetPID();
            return true;
        }

        long now = System.nanoTime();
        double dt = (pidTimer == 0) ? 0.02 : (now - pidTimer) / 1e9;
        pidTimer = now;

        dt = Range.clip(dt, 0.005, 0.05);

        double derivada = (error - errorAnterior) / dt;
        errorAnterior = error;

        double integralCandidate = integralError + (error * dt);
        integralCandidate = Range.clip(integralCandidate, -integralMax, integralMax);

        double salidaSinClip = (kP * error) + (kI * integralCandidate) + (kD * derivada);
        double salida = Range.clip(salidaSinClip, -SERVO_RANGE, SERVO_RANGE);

        boolean saturado = Math.abs(salidaSinClip) > SERVO_RANGE;

        if (!saturado || Math.signum(error) != Math.signum(salida)) {
            integralError = integralCandidate;
        }

        double pos = Range.clip(SERVO_CENTER + salida, 0.0, 1.0);

        Lanzador_I.setPosition(pos);
        Lanzador_D.setPosition(pos);

        correccion = pos;

        return false;
    }

    private boolean pasoTiempo(long start, long ms) {
        return ((System.nanoTime() - start) / 1_000_000) >= ms;
    }

    private void leerTagPre() {
        LLResult res = limaluz.getLatestResult();

        if (res != null && res.isValid() && res.getStaleness() <= 10) {
            List<LLResultTypes.FiducialResult> tags = res.getFiducialResults();
            if (tags != null && !tags.isEmpty()) {
                leidoID = tags.get(0).getFiducialId();
            }
        }

        if (pasoTiempo(preStart, p5Ms)) {
            limaluz.pipelineSwitch(pipelineAlinear);
            centrarLanzadorServos();
            resetPID();

            preEstado = preAlinear;
            alignStart = System.nanoTime();
        }
    }

    private void preUpdate() {
        switch (preEstado) {

            case preLeerTagObelisco:
                leerTagPre();
                break;

            case preAlinear:
                if (alinear()) {
                    centrarLanzadorServos();
                    alignStart = 0;
                    preEstado = preListo;
                }
                break;

            case preListo:
            default:
                break;
        }
    }

    @Override
    public void init() {

        Lanzador_D = hardwareMap.get(Servo.class, "Lanzador_D");
        Lanzador_I = hardwareMap.get(Servo.class, "Lanzador_I");

        limaluz = hardwareMap.get(Limelight3A.class, "limaluz");
        limaluz.setPollRateHz(90);
        limaluz.pipelineSwitch(pipelineLeer);
        limaluz.start();

        centrarLanzadorServos();

        preEstado = preLeerTagObelisco;
        resetPID();

        preStart = 0;
        alignStart = 0;
    }

    @Override
    public void start() {
        limaluz.pipelineSwitch(pipelineLeer);
        leidoID = -1;

        preEstado = preLeerTagObelisco;
        preStart = System.nanoTime();
        alignStart = 0;

        resetPID();
    }

    @Override
    public void loop() {
        preUpdate();

        telemetry.addData("Tag (P5)", leidoID);
        telemetry.addData("Corr", correccion);
        telemetry.addData("Estado", preEstado);
        telemetry.update();
    }

    @Override
    public void stop() {
        centrarLanzadorServos();
        if (limaluz != null) {
            limaluz.pipelineSwitch(pipelineLeer);
            limaluz.stop();
        }
    }
}

class PinpointTest extends OpMode {

    private GoBildaPinpointDriver pinpoint;
    private Servo Direccion;

    double goalY = 332.74;
    double goalX = 40.64;
    double xb;

    double H2 = 0.85675;
    double Thetai = -45;
    double Theta0 = 0;
    double direccion = 0;

    private int calcularVelocidadDisparo() {
        pinpoint.update();

        double pos_Y = pinpoint.getPosY(DistanceUnit.CM);
        double pos_X = pinpoint.getPosX(DistanceUnit.CM);

        double deltaX = goalX - pos_X;
        double deltaY = goalY - pos_Y;

        xb = (Math.sqrt(deltaX * deltaX + deltaY * deltaY)) / 100.0;

        Theta0 = Math.atan((2 * H2 / xb) - Math.tan(Math.toRadians(Thetai)));
        direccion = ((68 - Math.toDegrees(Theta0)) / 38) * 0.2;

        direccion = Range.clip(direccion, 0.0, 1.0);
        Direccion.setPosition(direccion);

        return (int) (793.60311 * Math.pow(1.19091, xb));
    }

    public void configurePinpoint() {
        pinpoint.setOffsets(-110 / 2.54, -160 / 2.54, DistanceUnit.MM);

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
    }

    @Override
    public void init() {
        Direccion = hardwareMap.get(Servo.class, "Direccion");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        configurePinpoint();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 56, 8, AngleUnit.DEGREES, 90));
        pinpoint.update();
    }

    @Override
    public void loop() {
        int v = calcularVelocidadDisparo();

        telemetry.addData("Y (cm)", pinpoint.getPosY(DistanceUnit.CM));
        telemetry.addData("X (cm)", pinpoint.getPosX(DistanceUnit.CM));
        telemetry.addData("Heading (deg)", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Distancia xb (m)", xb);
        telemetry.addData("Servo Direccion", direccion);
        telemetry.addData("Velocidad calc", v);
        telemetry.update();
    }
}

class ColorDetectorTest extends OpMode {

    private PredominantColorProcessor sensorColor;
    private VisionPortal portalColor;

    private DcMotorEx Recogedor, M_licuadora;

    private static final int recolectorEmpieza = 0;
    private static final int recolectoCambiaLicuadora = 1;
    private int recolectState = recolectorEmpieza;

    private long recolectTimer = 0;
    private static final long esperarRecolectMs = 250;

    private static final long colorEstableMs = 80;
    private static final long nadaEstableMs = 60;
    private static final long enfriarGuardadoMs = 250;

    String detectado = "Nada";
    String detectadoRaw = "Nada";

    private String candidatoDetect = "Nada";
    private long candidatoTimer = 0;
    private long nadaTimer = 0;

    private boolean armadoNuevaPieza = true;
    private long ultimoGuardadoTimer = 0;

    private boolean guardadoPendiente = false;
    private String guardadoPendienteColor = "Nada";

    private static final int SLOT_X = 0;
    private static final int SLOT_A = 1;
    private static final int SLOT_B = 2;

    String slotA = "Nada";
    String slotB = "Nada";
    String slotX = "Nada";

    private int slotActual = SLOT_X;
    private int slotObjetivo = SLOT_X;

    public int posX = 0;
    public int posA = 664;
    public int posB = -664;

    private static final int countsPorPasoLicuadora = 664;
    private static final int countsPorVueltaLicuadora = countsPorPasoLicuadora * 3;

    private final PIDController pidLicuadora = new PIDController(0.01, 0.0, 0.0003);
    private int licuadoraTargetCounts = 0;
    private static final int toleranciaLicuadora = 8;

    // ---------------- LICUADORA ----------------

    private int countsDeSlot(int slot) {
        if (slot == SLOT_A) return posA;
        if (slot == SLOT_B) return posB;
        return posX;
    }

    private int normalizarCountsVuelta(int countsBase) {
        int m = countsBase % countsPorVueltaLicuadora;
        if (m < 0) m += countsPorVueltaLicuadora;
        return m;
    }

    private int targetMasCercanoAEncoder(int countsBase) {
        int current = M_licuadora.getCurrentPosition();
        int baseNorm = normalizarCountsVuelta(countsBase);

        double kReal = (current - baseNorm) / (double) countsPorVueltaLicuadora;
        long k = Math.round(kReal);

        long target = (long) baseNorm + k * (long) countsPorVueltaLicuadora;
        return (int) target;
    }

    private int siguienteSlot(int slot) {
        if (slot == SLOT_X) return SLOT_A;
        if (slot == SLOT_A) return SLOT_B;
        return SLOT_X;
    }

    private boolean slotVacio(int slot) {
        if (slot == SLOT_X) return "Nada".equals(slotX);
        if (slot == SLOT_A) return "Nada".equals(slotA);
        return "Nada".equals(slotB);
    }

    private boolean slotTieneAlgo(int slot) {
        return !slotVacio(slot);
    }

    private int siguienteSlotConAlgo(int base) {
        int s1 = siguienteSlot(base);
        if (slotTieneAlgo(s1)) return s1;
        int s2 = siguienteSlot(s1);
        if (slotTieneAlgo(s2)) return s2;
        int s3 = siguienteSlot(s2);
        if (slotTieneAlgo(s3)) return s3;
        return s1;
    }

    private boolean preferirSlotEnEmpate(int baseSlot, int candidatoNuevo, int candidatoActual) {
        int s = baseSlot;
        for (int i = 0; i < 3; i++) {
            s = siguienteSlot(s);
            if (s == candidatoNuevo) return true;
            if (s == candidatoActual) return false;
        }
        return false;
    }

    private int slotVacioMasCercano(int baseSlot) {
        int current = M_licuadora.getCurrentPosition();

        int mejorSlot = -1;
        long mejorDist = Long.MAX_VALUE;

        int[] lista = new int[]{SLOT_X, SLOT_A, SLOT_B};

        for (int s : lista) {
            if (!slotVacio(s)) continue;

            int target = targetMasCercanoAEncoder(countsDeSlot(s));
            long dist = Math.abs((long) target - (long) current);

            if (dist < mejorDist) {
                mejorDist = dist;
                mejorSlot = s;
            } else if (dist == mejorDist && mejorSlot != -1) {
                if (preferirSlotEnEmpate(baseSlot, s, mejorSlot)) {
                    mejorSlot = s;
                }
            }
        }

        return mejorSlot;
    }

    private void irASlot(int slot) {
        slotObjetivo = slot;
        int base = countsDeSlot(slot);
        licuadoraTargetCounts = targetMasCercanoAEncoder(base);
    }

    private boolean enPosicion() {
        int err = licuadoraTargetCounts - M_licuadora.getCurrentPosition();
        return Math.abs(err) <= toleranciaLicuadora;
    }

    private void licuadoraPID() {
        int current = M_licuadora.getCurrentPosition();
        int error = licuadoraTargetCounts - current;

        if (Math.abs(error) <= toleranciaLicuadora) {
            M_licuadora.setPower(0);
            pidLicuadora.reset();
            slotActual = slotObjetivo;
            return;
        }

        double power = pidLicuadora.update(licuadoraTargetCounts, current);
        power = Range.clip(power, -1.0, 1.0);
        M_licuadora.setPower(power);
    }

    private void limpiarSlotActual() {
        if (slotActual == SLOT_X) slotX = "Nada";
        else if (slotActual == SLOT_A) slotA = "Nada";
        else slotB = "Nada";
    }

    private int contarSlots() {
        int c = 0;
        if (!"Nada".equals(slotX)) c++;
        if (!"Nada".equals(slotA)) c++;
        if (!"Nada".equals(slotB)) c++;
        return c;
    }

    private boolean guardarEnSlotActual(String color) {
        if ("Nada".equals(color)) return false;

        if (slotActual == SLOT_X) {
            if ("Nada".equals(slotX)) {
                slotX = color;
                return true;
            }
        } else if (slotActual == SLOT_A) {
            if ("Nada".equals(slotA)) {
                slotA = color;
                return true;
            }
        } else {
            if ("Nada".equals(slotB)) {
                slotB = color;
                return true;
            }
        }
        return false;
    }

    private void guardarEnMemoria() {
        long tNow = System.nanoTime();
        int contadorSlots = contarSlots();

        boolean puedoArmar = !"Nada".equals(detectado)
                && armadoNuevaPieza
                && (contadorSlots < 3)
                && ((tNow - ultimoGuardadoTimer) / 1_000_000 >= enfriarGuardadoMs);

        if (puedoArmar && !guardadoPendiente) {
            guardadoPendiente = true;
            guardadoPendienteColor = detectado;
        }

        switch (recolectState) {
            case recolectorEmpieza:
                if (guardadoPendiente && enPosicion()) {

                    if (!slotVacio(slotActual)) {
                        int sig = slotVacioMasCercano(slotActual);
                        if (sig != -1 && sig != slotActual) {
                            irASlot(sig);
                            recolectTimer = tNow;
                            recolectState = recolectoCambiaLicuadora;
                        }
                        break;
                    }

                    boolean guardo = guardarEnSlotActual(guardadoPendienteColor);

                    if (guardo) {
                        armadoNuevaPieza = false;
                        ultimoGuardadoTimer = tNow;
                        guardadoPendiente = false;
                        guardadoPendienteColor = "Nada";

                        int sig = slotVacioMasCercano(slotActual);
                        if (sig != -1) {
                            irASlot(sig);
                            recolectTimer = tNow;
                            recolectState = recolectoCambiaLicuadora;
                        }
                    } else {
                        int sig = slotVacioMasCercano(slotActual);
                        if (sig != -1 && sig != slotActual) {
                            irASlot(sig);
                            recolectTimer = tNow;
                            recolectState = recolectoCambiaLicuadora;
                        }
                    }
                }
                break;

            case recolectoCambiaLicuadora: {
                long ms = (tNow - recolectTimer) / 1_000_000;
                if (ms > esperarRecolectMs) {
                    recolectState = recolectorEmpieza;
                }
                break;
            }

            default:
                recolectState = recolectorEmpieza;
                break;
        }
    }

    private void resetSlots() {
        slotA = "Nada";
        slotB = "Nada";
        slotX = "Nada";

        detectado = "Nada";
        detectadoRaw = "Nada";
        candidatoDetect = "Nada";

        armadoNuevaPieza = true;
        recolectState = recolectorEmpieza;

        guardadoPendiente = false;
        guardadoPendienteColor = "Nada";

        slotActual = SLOT_X;
        irASlot(SLOT_X);
    }

    // ---------------- COLOR ----------------

    private void camaraColor() {
        sensorColor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.25, 0.25, 0.25, -0.25))
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE
                )
                .build();

        portalColor = new VisionPortal.Builder()
                .addProcessor(sensorColor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam1"))
                .build();
    }

    private void detectarColor() {
        detectadoRaw = "Nada";

        if (portalColor != null && sensorColor != null) {
            PredominantColorProcessor.Result resultColor = sensorColor.getAnalysis();

            if (resultColor != null) {
                if (resultColor.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN) {
                    detectadoRaw = "Verde";
                } else if (resultColor.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) {
                    detectadoRaw = "Purpura";
                }
            }
        }

        long tNow = System.nanoTime();

        if (!detectadoRaw.equals(candidatoDetect)) {
            candidatoDetect = detectadoRaw;
            candidatoTimer = tNow;
        }

        long candidatoMs = (tNow - candidatoTimer) / 1_000_000;

        if (candidatoMs >= colorEstableMs) {
            if (!detectado.equals(candidatoDetect)) {
                detectado = candidatoDetect;
                if ("Nada".equals(detectado)) {
                    nadaTimer = tNow;
                }
            }
        }

        if ("Nada".equals(detectado)) {
            long msNada = (tNow - nadaTimer) / 1_000_000;
            if (msNada >= nadaEstableMs) {
                armadoNuevaPieza = true;
            }
        }
    }

    @Override
    public void init() {
        Recogedor = hardwareMap.get(DcMotorEx.class, "Recogedor");
        M_licuadora = hardwareMap.get(DcMotorEx.class, "licuadora");

        M_licuadora.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_licuadora.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_licuadora.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        irASlot(SLOT_X);

        camaraColor();

        long tInit = System.nanoTime();
        candidatoTimer = tInit;
        nadaTimer = tInit;
        ultimoGuardadoTimer = 0;

        resetSlots();
    }

    @Override
    public void start() {
        resetSlots();
    }

    @Override
    public void loop() {
        licuadoraPID();

        Recogedor.setPower(1.0);

        detectarColor();
        guardarEnMemoria();

        telemetry.addData("DetectadoRaw", detectadoRaw);
        telemetry.addData("DetectadoEstable", detectado);

        telemetry.addData("slotX", slotX);
        telemetry.addData("slotA", slotA);
        telemetry.addData("slotB", slotB);

        telemetry.addData("slotActual", slotActual);
        telemetry.addData("slotObjetivo", slotObjetivo);
        telemetry.addData("targetCounts", licuadoraTargetCounts);
        telemetry.addData("encoder", M_licuadora.getCurrentPosition());

        telemetry.update();
    }

    @Override
    public void stop() {
        if (Recogedor != null) Recogedor.setPower(0.0);
        if (M_licuadora != null) M_licuadora.setPower(0.0);

        if (portalColor != null) {
            portalColor.close();
            portalColor = null;
        }
    }

    public class PIDController {
        double kP, kI, kD;
        double integral = 0;
        double lastError = 0;
        long lastTime;

        private PIDController(double p, double i, double d) {
            kP = p;
            kI = i;
            kD = d;
            lastTime = System.currentTimeMillis();
        }

        private double update(double target, double current) {
            long now = System.currentTimeMillis();
            double dt = (now - lastTime) / 1000.0;
            lastTime = now;

            dt = Math.max(dt, 0.001);

            double error = target - current;
            integral += error * dt;

            double derivative = (error - lastError) / dt;
            lastError = error;

            return kP * error + kI * integral + kD * derivative;
        }

        public void reset() {
            integral = 0;
            lastError = 0;
            lastTime = System.currentTimeMillis();
        }
    }
}