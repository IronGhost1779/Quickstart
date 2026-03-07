package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="Auto Azul Goal")
public class AutoRojoGoal extends OpMode {

    // ---------------- HARDWARE ----------------

    private DcMotorEx M_lanzador1, M_lanzador2, Recogedor, M_licuadora;

    private Servo Direccion, Levantador;
    private Servo Lanzador_I, Lanzador_D;

    private GoBildaPinpointDriver pinpoint;

    private PredominantColorProcessor sensorColor;
    private VisionPortal portalColor;

    private Limelight3A limaluz;

    // ---------------- LIMELIGHT ----------------

    private static final int pipelineLeer = 5;
    private static final int pipelineAlinear = 6;

    public int leidoID = -1;

    // ---------------- MOR ----------------

    ArrayList<Boolean> lista_MOR = new ArrayList<Boolean>();

    private boolean morListo = false;
    private boolean licuadoraListaInicio = false;

    // ---------------- SLOTS / LICUADORA ----------------

    private static final int SLOT_X = 0;
    private static final int SLOT_A = 1;
    private static final int SLOT_B = 2;

    String slotA = "Purpura";
    String slotB = "Verde";
    String slotX = "Purpura";

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

    // ---------------- DISPARO ----------------

    double goalY = 332.74;
    double goalX = 40.64;
    double xb;

    double H2 = 0.85675;
    double Thetai = -45; // 45
    double Theta0 = 0;
    double direccion = 0;

    // ---------------- CONTROL (ALINEACIÓN) ----------------

    private static final double kP = 0.008; // 0.008
    private static final double kD = 0.0006;
    private static final double kI = 0.0015;
    private double errorAnterior = 0.0;
    private double integralError = 0.0;
    private static final double integralMax = 250.0;
    private long pidTimer = 0;

    private static final double SERVO_CENTER = 0.5;
    private static final double SERVO_RANGE = 0.25;

    private double correccion = SERVO_CENTER;

    private static final double tolerancia = 0.04;

    private static final long p5Ms = 80;
    private static final long timeutAlinearMs = 1500; // 2300

    private long preStart = 0;
    private long alignStart = 0;
    private static final double headingPre = Math.toRadians(0);
    private static final double deltaRotacionInch = 0.08;

    // ---------------- POWER CONTROL ----------------

    private static final double MAX_POWER_NORMAL = 1.0;
    private static final double MAX_POWER_RECOLECT = 0.215; // 0.195
    private static final double MAX_POWER_ROTACION = 0.8;

    // ---------------- PATH STATES ----------------

    private static final int INICIO_A_RECOLECCION = 0;
    private static final int RECOLECCION = 1;

    private static final int RECOLECCION1_A_LANZAMIENTO1 = 2;
    private static final int ALINEA1 = 3;
    private static final int DISPARA1 = 4;

    private static final int LANZAMIENTO1_A_RECOLECCION2 = 5;
    private static final int RECOLECCION2 = 6;
    private static final int RECOLECCION2_A_LANZAMIENTO2 = 7;
    private static final int ALINEA2 = 8;
    private static final int DISPARA2 = 9;

    private static final int LANZAMIENTO2_A_RECOLECCION3 = 10;
    private static final int RECOLECCION3 = 11;
    private static final int RECOLECCION3_A_LANZAMIENTO3 = 12;
    private static final int ALINEA3 = 13;
    private static final int DISPARA3 = 14;

    private static final int FIN = 15;
    private static final int DONE = 16;

    private static final int ESPERA_PRE = 17;

    private static final int DISPARO1_A_INTERMEDIO2 = 18;
    private static final int RECOLECCION_FILA2 = 19;

    private int estado = INICIO_A_RECOLECCION;

    // ---------------- PRE-ESTADOS ----------------

    private static final int preLeerTagObelisco = 0;
    private static final int preRotar130 = 1;
    private static final int preAlinear = 2;
    private static final int preDisparar = 3;
    private static final int preListo = 4;

    private int preEstado = preListo;

    // ---------------- TIMERS DISPARO ----------------

    private static final long bajarMs = 380;
    private static final long cambiarLicuadoraMs = 690;
    private static final long esperarAntesDeSubirMs = 990;

    private long slotTimer = 0;

    // ---------------- DISPARO FSM ----------------

    private static final int disparadorEmpieza = 0;
    private static final int disparadorSube = 1;
    private static final int disparadorBaja = 2;
    private static final int disparadorFin = 3;
    private static final int disparadorEsperarSlot = 4;

    private int shootState = disparadorEmpieza;

    private static final int NUM_DISPAROS = 3;
    private int disparosHechos = 0;
    private long shootTimer = 0;

    private boolean bajadoLevantador = false;

    // ---------------- RECOLECCIÓN FSM ----------------

    private static final int recolectorEmpieza = 0;
    private static final int recolectoCambiaLicuadora = 1;
    private int recolectState = recolectorEmpieza;

    private long recolectTimer = 0;
    private static final long esperarRecolectMs = 250;

    private static final long colorEstableMs = 80; // 100
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

    private boolean recolectandoPath = false;

    // ---------------- FOLLOWER / PATHS ----------------

    private Follower follower;
    private Timer pathTimer;

    private final Pose posicionInicio = new Pose(newNumber(21.744094)
            , 123.377953, Math.toRadians(newNumber(146)));
    private final Pose lecturaApriltag = new Pose(newNumber(52.62185257302387)
            , 90.49706252136238, Math.toRadians(newNumber(155)));
    private final Pose posicionIntermedio1 = new Pose(newNumber(49.17504332755635)
            , 90.49706252136238, Math.toRadians(newNumber(180)));
    private final Pose posicionRecoleccion1 = new Pose(newNumber(25.59774696707106)
            , 90.49706252136238, Math.toRadians(newNumber(180)));
    private final Pose posicionLanzamiento1 = new Pose(newNumber(52.62185257302387)
            , 90.49706252136238, Math.toRadians(newNumber(180)));
    private final Pose posicionIntermedio2 = new Pose(newNumber(48.45580589254766)
            , 68.69844020797226, Math.toRadians(newNumber(180)));
    private final Pose posicionRecoleccion2 = new Pose(newNumber(26.73483535528596)
            , 68.69844020797226, Math.toRadians(newNumber(180)));
    private final Pose posicionLanzamiento2 = new Pose(newNumber(58.14384748700175)
            , 84.84228769497399, Math.toRadians(newNumber(180)));
    private final Pose posicionFin = new Pose(newNumber(62.270363951473136)
            , 110.043327556325835, Math.toRadians(newNumber(90)));

    private PathChain inicioARecoleccion1;

    public void buildPaths() {
        inicioARecoleccion1 = follower.pathBuilder()
                .addPath(new BezierLine(posicionInicio, lecturaApriltag))
                .setLinearHeadingInterpolation(posicionInicio.getHeading(), lecturaApriltag.getHeading())
                .build();
    }

    // ---------------- RECOLECCIÓN DINÁMICA ----------------

    private PathChain buildRecoleccion(double xFinal, double headingConstante) {
        pinpoint.update();
        double xInicio = pinpoint.getPosX(DistanceUnit.INCH);
        double yInicio = pinpoint.getPosY(DistanceUnit.INCH);

        Pose inicio = new Pose(xInicio, yInicio, headingConstante);
        Pose fin = new Pose(xFinal, yInicio, headingConstante);

        return follower.pathBuilder()
                .addPath(new BezierLine(inicio, fin))
                .setLinearHeadingInterpolation(headingConstante, headingConstante)
                .build();
    }

    private PathChain buildLineaHacia(Pose destino, double headingInicio, double headingFin) {
        pinpoint.update();
        double xInicio = pinpoint.getPosX(DistanceUnit.INCH);
        double yInicio = pinpoint.getPosY(DistanceUnit.INCH);

        Pose inicio = new Pose(xInicio, yInicio, headingInicio);

        return follower.pathBuilder()
                .addPath(new BezierLine(inicio, destino))
                .setLinearHeadingInterpolation(headingInicio, headingFin)
                .build();
    }

    private PathChain buildRotacionEnEje(double headingTarget) {
        pinpoint.update();
        double xInicio = pinpoint.getPosX(DistanceUnit.INCH);
        double yInicio = pinpoint.getPosY(DistanceUnit.INCH);
        double headingInicio = Math.toRadians(pinpoint.getHeading(AngleUnit.DEGREES));

        Pose inicio = new Pose(xInicio, yInicio, headingInicio);
        Pose finRot = new Pose(xInicio + deltaRotacionInch, yInicio, headingTarget);

        return follower.pathBuilder()
                .addPath(new BezierLine(inicio, finRot))
                .setLinearHeadingInterpolation(headingInicio, headingTarget)
                .build();
    }

    // ---------------- ALINEACIÓN ----------------

    private void iniciarAlineacion() {
        limaluz.pipelineSwitch(pipelineAlinear);
        centrarLanzadorServos();
        pidTimer = 0;
        errorAnterior = 0.0;
        integralError = 0.0;
        alignStart = System.nanoTime();
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
            pidTimer = 0;
            errorAnterior = 0.0;
            integralError = 0.0;
            return false;
        }

        double error = -tx;

        if (Math.abs(error) < tolerancia) {
            pidTimer = 0;
            errorAnterior = 0.0;
            integralError = 0.0;
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

    private void centrarLanzadorServos() {
        Lanzador_I.setPosition(SERVO_CENTER);
        Lanzador_D.setPosition(SERVO_CENTER);
        correccion = SERVO_CENTER;
    }

    private boolean pasoTiempo(long start, long ms) {
        return ((System.nanoTime() - start) / 1_000_000) >= ms;
    }

    // ---------------- PATH UPDATE ----------------

    private void cortarPathRecoleccionActual() {
        if (follower != null && follower.isBusy()) {
            follower.breakFollowing();
        }
    }

    private void irALanzamiento1Dyn() {
        recolectandoPath = false;
        follower.setMaxPower(MAX_POWER_NORMAL);

        pinpoint.update();
        double xInicio = pinpoint.getPosX(DistanceUnit.INCH);
        double yInicio = pinpoint.getPosY(DistanceUnit.INCH);
        double headingInicio = Math.toRadians(pinpoint.getHeading(AngleUnit.DEGREES));

        Pose inicio = new Pose(xInicio, yInicio, headingInicio);

        Pose lanzamiento1Dyn = new Pose(
                posicionLanzamiento1.getX(),
                yInicio,
                posicionLanzamiento1.getHeading()
        );

        PathChain aLanzamiento1Dyn = follower.pathBuilder()
                .addPath(new BezierLine(inicio, lanzamiento1Dyn))
                .setLinearHeadingInterpolation(headingInicio, lanzamiento1Dyn.getHeading())
                .build();

        follower.followPath(aLanzamiento1Dyn, true);
        setPathState(ALINEA1);
    }

    private void irALanzamiento2() {
        recolectandoPath = false;
        follower.setMaxPower(MAX_POWER_NORMAL);

        pinpoint.update();
        double xInicio = pinpoint.getPosX(DistanceUnit.INCH);
        double yInicio = pinpoint.getPosY(DistanceUnit.INCH);
        double headingInicio = Math.toRadians(pinpoint.getHeading(AngleUnit.DEGREES));

        Pose inicio = new Pose(xInicio, yInicio, headingInicio);

        PathChain aLanzamiento2 = follower.pathBuilder()
                .addPath(new BezierLine(inicio, posicionLanzamiento2))
                .setLinearHeadingInterpolation(headingInicio, posicionLanzamiento2.getHeading())
                .build();

        follower.followPath(aLanzamiento2, true);
        setPathState(RECOLECCION2_A_LANZAMIENTO2);
    }

    public void statePathUpdate() {
        switch (estado) {

            case INICIO_A_RECOLECCION:

                recolectandoPath = false;
                follower.setMaxPower(MAX_POWER_NORMAL);
                follower.followPath(inicioARecoleccion1, true);
                setPathState(RECOLECCION);
                break;

            case RECOLECCION:
                if (!follower.isBusy()) {
                    iniciarPreDespuesDePrimerPath();
                    setPathState(ESPERA_PRE);
                }
                break;

            case ESPERA_PRE:
                if (preEstado == preListo) {
                    if (contarSlots() == 3) {
                        break;
                    }

                    recolectandoPath = false;
                    follower.setMaxPower(MAX_POWER_NORMAL);

                    pinpoint.update();
                    double yBase = pinpoint.getPosY(DistanceUnit.INCH);

                    Pose intermedioDyn = new Pose(
                            posicionIntermedio1.getX(),
                            yBase,
                            posicionIntermedio1.getHeading()
                    );

                    double headingInicio = Math.toRadians(pinpoint.getHeading(AngleUnit.DEGREES));

                    PathChain aIntermedioDyn = buildLineaHacia(
                            intermedioDyn,
                            headingInicio,
                            intermedioDyn.getHeading()
                    );

                    follower.followPath(aIntermedioDyn, true);
                    setPathState(RECOLECCION1_A_LANZAMIENTO1);
                }
                break;

            case RECOLECCION1_A_LANZAMIENTO1:
                if (!follower.isBusy()) {
                    recolectandoPath = true;
                    follower.setMaxPower(MAX_POWER_RECOLECT);

                    PathChain recoleccionDyn = buildRecoleccion(
                            posicionRecoleccion1.getX(),
                            posicionRecoleccion1.getHeading()
                    );

                    follower.followPath(recoleccionDyn, true);

                    setPathState(LANZAMIENTO1_A_RECOLECCION2);
                }
                break;

            case LANZAMIENTO1_A_RECOLECCION2:
                if (contarSlots() == 3) {
                    cortarPathRecoleccionActual();
                    irALanzamiento1Dyn();
                    break;
                }

                if (!follower.isBusy()) {
                    irALanzamiento1Dyn();
                }
                break;

            case ALINEA1:
                if (!follower.isBusy()) {
                    shootState = disparadorEmpieza;
                    disparosHechos = 0;
                    bajadoLevantador = false;

                    calcularPosicion(0);
                    setPathState(DISPARA1);
                }
                break;

            case DISPARA1:
                if (dispararEnPatron()) {
                    recolectandoPath = false;
                    follower.setMaxPower(MAX_POWER_NORMAL);

                    pinpoint.update();
                    double headingInicioD1 = Math.toRadians(pinpoint.getHeading(AngleUnit.DEGREES));

                    PathChain aIntermedio2 = buildLineaHacia(
                            posicionIntermedio2,
                            headingInicioD1,
                            posicionIntermedio2.getHeading()
                    );

                    follower.followPath(aIntermedio2, true);
                    setPathState(DISPARO1_A_INTERMEDIO2);
                }
                break;

            case DISPARO1_A_INTERMEDIO2:
                if (!follower.isBusy()) {
                    recolectandoPath = true;
                    follower.setMaxPower(MAX_POWER_RECOLECT);

                    PathChain recoleccionFila2Dyn = buildRecoleccion(
                            posicionRecoleccion2.getX(),
                            posicionRecoleccion2.getHeading()
                    );

                    follower.followPath(recoleccionFila2Dyn, true);
                    setPathState(RECOLECCION_FILA2);
                }
                break;

            case RECOLECCION_FILA2:
                if (contarSlots() == 3) {
                    cortarPathRecoleccionActual();
                    irALanzamiento2();
                    break;
                }

                if (!follower.isBusy()) {
                    irALanzamiento2();
                }
                break;

            case RECOLECCION2_A_LANZAMIENTO2:
                if (!follower.isBusy()) {
                    shootState = disparadorEmpieza;
                    disparosHechos = 0;
                    bajadoLevantador = false;

                    calcularPosicion(0);
                    setPathState(DISPARA2);
                }
                break;

            case DISPARA2:
                if (dispararEnPatron()) {
                    recolectandoPath = false;
                    follower.setMaxPower(MAX_POWER_NORMAL);

                    pinpoint.update();
                    double headingInicioFin = Math.toRadians(pinpoint.getHeading(AngleUnit.DEGREES));

                    PathChain aFin = buildLineaHacia(
                            posicionFin,
                            headingInicioFin,
                            posicionFin.getHeading()
                    );

                    follower.followPath(aFin, true);
                    setPathState(FIN);
                }
                break;

            case FIN:
                if (!follower.isBusy()) {
                    setPathState(DONE);
                }
                break;

            case DONE:
                follower.setMaxPower(MAX_POWER_NORMAL);
                recolectandoPath = false;
                if (!follower.isBusy()) {
                    follower.breakFollowing();
                }
                telemetry.addLine("DONE");
                telemetry.update();
                break;

            default:
                break;
        }
    }

    public void setPathState(int nuevoEstado) {
        estado = nuevoEstado;
        if (pathTimer != null) pathTimer.resetTimer();

        if (estado == ALINEA1 || estado == ALINEA2 || estado == ALINEA3) {
            alignStart = 0;
        }
    }

    // ---------------- PRE-LOGICA ----------------

    private void iniciarPreDespuesDePrimerPath() {
        limaluz.pipelineSwitch(pipelineLeer);
        leidoID = -1;

        preEstado = preLeerTagObelisco;
        preStart = System.nanoTime();
        alignStart = 0;

        pidTimer = 0;
        errorAnterior = 0.0;
        integralError = 0.0;

        shootState = disparadorEmpieza;
        disparosHechos = 0;
        bajadoLevantador = false;
    }

    private void preUpdate() {
        switch (preEstado) {

            case preLeerTagObelisco:
                leerTagPre();
                break;

            case preRotar130:
                if (!follower.isBusy()) {
                    iniciarAlineacion();
                    preEstado = preAlinear;
                }
                break;

            case preAlinear:
                if (alinear() || pasoTiempo(alignStart, timeutAlinearMs)) {
                    alignStart = 0;

                    preEstado = preDisparar;
                    shootState = disparadorEmpieza;
                    disparosHechos = 0;
                    bajadoLevantador = false;

                    calcularPosicion(0);
                }
                break;

            case preDisparar:
                if (dispararEnPatron()) {
                    preEstado = preListo;
                }
                break;

            case preListo:
                break;
        }
    }

    private void leerTagPre() {
        LLResult res = limaluz.getLatestResult();

        if (res != null && res.isValid() && res.getStaleness() <= 10) {
            List<LLResultTypes.FiducialResult> tags = res.getFiducialResults();
            if (tags != null && !tags.isEmpty()) {
                leidoID = tags.get(0).getFiducialId();
                determinarMor();

                if (morListo && !licuadoraListaInicio) {
                    prepararLicuadoraParaDisparo(0);
                    licuadoraListaInicio = true;
                }
            }
        }

        if (pasoTiempo(preStart, p5Ms)) {
            limaluz.pipelineSwitch(pipelineAlinear);
            centrarLanzadorServos();

            pidTimer = 0;
            errorAnterior = 0.0;
            integralError = 0.0;

            follower.setMaxPower(MAX_POWER_ROTACION);

            PathChain rotar = buildRotacionEnEje(headingPre);
            follower.followPath(rotar, true);

            preEstado = preRotar130;
        }
    }

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

        int[] lista = new int[] {SLOT_X, SLOT_A, SLOT_B};

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

        if (M_licuadora != null) {
            licuadoraTargetCounts = targetMasCercanoAEncoder(base);
        } else {
            licuadoraTargetCounts = base;
        }
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
        slotA = "Purpura";
        slotB = "Verde";
        slotX = "Purpura";

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

    private void recolectar(){
        if (recolectandoPath) {
            Recogedor.setPower(-1);
            detectarColor();
            guardarEnMemoria();
        } else {
            Recogedor.setPower(0.0);
            recolectState = recolectorEmpieza;
            armadoNuevaPieza = true;
            guardadoPendiente = false;
            guardadoPendienteColor = "Nada";
        }
    }

    // ---------------- DISPARO ----------------

    private int calcularVelocidadDisparo() {
        pinpoint.update();

        double pos_Y = pinpoint.getPosY(DistanceUnit.CM);
        double pos_X = pinpoint.getPosX(DistanceUnit.CM);

        double deltaX = goalX - pos_X;
        double deltaY = goalY - pos_Y;

        xb = (Math.sqrt(deltaX * deltaX + deltaY * deltaY)) / 100.0;

        Theta0 = Math.atan((2*H2/xb) - Math.tan(Math.toRadians(Thetai)));
        direccion = ((68 - Math.toDegrees(Theta0))/38)*0.2;

        Direccion.setPosition(direccion);

        // return (int) (829.40011 * Math.pow(1.17945, xb));
        return (int) (969.5520984852944 * Math.pow(xb, 0.2841685069817));
    }

    private int slotDeColor(String color) {
        if (color == null) return -1;
        if (color.equals(slotX)) return SLOT_X;
        if (color.equals(slotA)) return SLOT_A;
        if (color.equals(slotB)) return SLOT_B;
        return -1;
    }

    private int slotParaDisparo(int pos, int base) {
        if (!morListo || lista_MOR.size() <= pos) {
            return siguienteSlotConAlgo(base);
        }

        boolean wantPurpura = lista_MOR.get(pos);
        String objetivo = wantPurpura ? "Purpura" : "Verde";

        int slotTarget = slotDeColor(objetivo);

        if (slotTarget == -1) {
            slotTarget = siguienteSlotConAlgo(base);
        }

        if (slotVacio(slotTarget)) {
            slotTarget = siguienteSlotConAlgo(base);
        }

        return slotTarget;
    }

    private void prepararLicuadoraParaDisparo(int pos) {
        int slotTarget = slotParaDisparo(pos, slotActual);
        irASlot(slotTarget);
    }

    private void calcularPosicion(int pos) {
        int slotTarget = slotParaDisparo(pos, slotActual);
        irASlot(slotTarget);
    }

    private boolean dispararEnPatron() {
        int v = (int) (calcularVelocidadDisparo());

        Recogedor.setPower(1);

        switch (shootState) {

            case disparadorEmpieza:

                long ms1 = (System.nanoTime()) / 1_000_000;

                if(ms1 > 400){
                    Recogedor.setPower(0);
                    shootTimer = System.nanoTime();
                    shootState = disparadorSube;
                    return false;
                }

            case disparadorSube: {
                double velActual = (M_lanzador1.getVelocity() + M_lanzador2.getVelocity())/2;
                long ms = (System.nanoTime() - shootTimer) / 1_000_000;

                if ((Math.abs(velActual - v) < 50 || ms > 2000) && enPosicion()) {
                    Levantador.setPosition(1.0);
                    shootTimer = System.nanoTime();
                    shootState = disparadorBaja;
                    bajadoLevantador = false;
                }
                return false;
            }

            case disparadorBaja: {
                long ms = (System.nanoTime() - shootTimer) / 1_000_000;

                if (!bajadoLevantador && ms > bajarMs) {
                    Levantador.setPosition(0.6);
                    limpiarSlotActual();
                    disparosHechos++;
                    bajadoLevantador = true;
                    shootTimer = System.nanoTime();
                    return false;
                }

                if (bajadoLevantador && ms > cambiarLicuadoraMs) {
                    if (disparosHechos >= NUM_DISPAROS) {
                        shootState = disparadorFin;
                        return true;
                    }

                    calcularPosicion(disparosHechos);

                    slotTimer = System.nanoTime();
                    shootState = disparadorEsperarSlot;
                    return false;
                }

                return false;
            }

            case disparadorEsperarSlot: {
                long ms = (System.nanoTime() - slotTimer) / 1_000_000;
                if (ms > esperarAntesDeSubirMs && enPosicion()) {
                    shootState = disparadorEmpieza;
                }
                return false;
            }

            case disparadorFin:
            default:
                return true;
        }
    }

    private void determinarMor() {
        if (leidoID != 21 && leidoID != 22 && leidoID != 23) return;

        if (lista_MOR.size() < 3) {
            lista_MOR.clear();
            lista_MOR.add(false);
            lista_MOR.add(false);
            lista_MOR.add(false);
        }

        if (leidoID == 21) {
            lista_MOR.set(0, false);
            lista_MOR.set(1, true);
            lista_MOR.set(2, true);
        } else if (leidoID == 22) {
            lista_MOR.set(0, true);
            lista_MOR.set(1, false);
            lista_MOR.set(2, true);
        } else {
            lista_MOR.set(0, true);
            lista_MOR.set(1, true);
            lista_MOR.set(2, false);
        }

        morListo = true;

        Storage.set(lista_MOR.get(0), lista_MOR.get(1), lista_MOR.get(2));
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
        detectado = "Nada";

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

    // ---------------- OPMODE ----------------

    @Override
    public void init() {
        estado = INICIO_A_RECOLECCION;
        pathTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        Direccion = hardwareMap.get(Servo.class, "Direccion");
        Levantador = hardwareMap.get(Servo.class, "Levantador");

        Lanzador_D = hardwareMap.get(Servo.class, "Lanzador_D");
        Lanzador_I = hardwareMap.get(Servo.class, "Lanzador_I");

        M_lanzador1 = hardwareMap.get(DcMotorEx.class, "M_lanzador1");
        M_lanzador2 = hardwareMap.get(DcMotorEx.class, "M_lanzador2");
        Recogedor = hardwareMap.get(DcMotorEx.class, "Recogedor");
        M_licuadora = hardwareMap.get(DcMotorEx.class, "licuadora");

        M_licuadora.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_licuadora.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_licuadora.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        irASlot(SLOT_X);

        M_lanzador1.setVelocityPIDFCoefficients(105, 0.0, 0.1, 12.0);
        M_lanzador2.setVelocityPIDFCoefficients(105, 0.0, 0.1, 12.0);

        limaluz = hardwareMap.get(Limelight3A.class, "limaluz");
        limaluz.setPollRateHz(90);
        limaluz.pipelineSwitch(pipelineLeer);
        limaluz.start();

        Direccion.setDirection(Servo.Direction.REVERSE);

        Levantador.setPosition(0.6);
        Direccion.setPosition(0);

        centrarLanzadorServos();

        configurePinpoint();
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH,
                posicionInicio.getX(), posicionInicio.getY(),
                AngleUnit.DEGREES, Math.toDegrees(posicionInicio.getHeading())
        ));
        pinpoint.update();

        buildPaths();
        follower.setPose(posicionInicio);

        camaraColor();

        lista_MOR.clear();
        lista_MOR.add(false);
        lista_MOR.add(false);
        lista_MOR.add(false);

        resetSlots();

        recolectandoPath = false;

        pidTimer = 0;
        errorAnterior = 0.0;
        preStart = 0;
        alignStart = 0;

        morListo = false;
        licuadoraListaInicio = false;

        preEstado = preListo;
    }

    @Override
    public void start() {
        setPathState(estado);

        limaluz.pipelineSwitch(pipelineLeer);
        leidoID = -1;

        preEstado = preListo;
        preStart = 0;
        alignStart = 0;

        shootState = disparadorEmpieza;
        disparosHechos = 0;

        resetSlots();

        recolectandoPath = false;

        pidTimer = 0;
        errorAnterior = 0.0;

        morListo = false;
        licuadoraListaInicio = false;
    }

    @Override
    public void loop() {
        int v = (int) (calcularVelocidadDisparo());

        M_lanzador1.setVelocity(v);
        M_lanzador2.setVelocity(v);

        licuadoraPID();
        calcularVelocidadDisparo();

        follower.update();
        statePathUpdate();
        preUpdate();

        recolectar();

        telemetry.addData("slotX", slotX);
        telemetry.addData("slotA", slotA);
        telemetry.addData("slotB", slotB);
        telemetry.addData("Tag (P5)", leidoID);
        telemetry.addData("Corr", correccion);
        telemetry.addData("Estado", estado);
        telemetry.addData("preEstado", preEstado);

        telemetry.update();

        Storage.set1((float) pinpoint.getPosX(DistanceUnit.INCH),
                (float) pinpoint.getPosY(DistanceUnit.INCH),
                (float) pinpoint.getHeading(AngleUnit.DEGREES));
    }

    public void configurePinpoint(){
        pinpoint.setOffsets(-110/2.54, -160/2.54, DistanceUnit.MM);

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
    }

    public double newNumber(double x){
        double res = 144.0 - x;
        return res;
    }

    public int newAngle(int x){
        int res = 180-x;
        return res;
    }

    // ---------------- PID CONTROLLER ----------------

    public class PIDController {
        double kP, kI, kD;
        double integral = 0;
        double lastError = 0;
        long lastTime;

        public PIDController(double p, double i, double d) {
            kP = p;
            kI = i;
            kD = d;
            lastTime = System.currentTimeMillis();
        }

        public double update(double target, double current) {
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