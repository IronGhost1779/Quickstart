package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class TeleOpFinal extends LinearOpMode {

    // Inicialización de motores, servos, camara, pinpoint y MOR

    GoBildaPinpointDriver pinpoint;
    private Limelight3A limaluz;
    ArrayList<Boolean> lista_MOR = new ArrayList<>();

    private DcMotorEx motor_Iz_Fr, motor_Iz_A, motor_De_Fr, motor_De_A;
    private DcMotorEx M_lanzador1, M_lanzador2, Recogedor, M_licuadora;
    private Servo Lanzador_I, Lanzador_D, Levantador, Direccion;

    // Variables para la lógica de los disparos
    private static final int NUM_DISPAROS = 3;
    private int disparosHechos = 0;
    private long shootTimer = 0;

    private boolean bajadoLevantador = false;
    boolean shooting3 = false;
    boolean shooting3ConPatron = false;

    private static final long bajarMs = 500;
    private static final long cambiarLicuadoraMs = 550;
    private static final long esperarAntesDeSubirMs = 1100;

    private long slotTimer = 0;

    int estadoLanzar = 0;
    boolean estadoLanzador = false;
    long inicioLanzador = 0;

    // Estados del disparador
    private static final int disparadorEmpieza = 0;
    private static final int disparadorSube = 1;
    private static final int disparadorBaja = 2;
    private static final int disparadorFin = 3;
    private static final int disparadorEsperarSlot = 4;
    private int shootState = disparadorEmpieza;

    // Estados del recolector
    private static final int recolectorEmpieza = 0;
    private static final int recolectoCambiaLicuadora = 1;
    private int recolectState = recolectorEmpieza;

    private long recolectTimer = 0;
    private static final long esperarRecolectMs = 300;

    // Todas las constantes y variables para el cálculo de la distancia y ángulo
    private static final double goalY = 332.74;
    private static final double goalX = 40.64;

    private static final double H2 = 0.85675;

    private static final double Thetai = -45;
    double Theta0 = 0;

    double xb;

    double direccion = 0;

    // Todas las variables de la lógica de la detección de color y guardado de color en los slots
    String detectado = "Nada";
    String detectadoRaw = "Nada";
    String slotA = "Nada";
    String slotB = "Nada";
    String slotX = "Nada";

    private PredominantColorProcessor sensorColor;
    private VisionPortal portalColor;

    private static final long colorEstableMs = 80;
    private static final long nadaEstableMs = 60;
    private static final long enfriarGuardadoMs = 250;

    private String candidatoDetect = "Nada";
    private long candidatoTimer = 0;
    private long nadaTimer = 0;

    private boolean armadoNuevaPieza = true;
    private long ultimoGuardadoTimer = 0;

    // ---- (INTAKE) ----
    private boolean guardadoPendiente = false;
    private String guardadoPendienteColor = "Nada";

    // Todas las variables para la lógica de los slots y la licuadora
    private static final int SLOT_X = 0;
    private static final int SLOT_A = 1;
    private static final int SLOT_B = 2;

    public int posX = 0;
    public double posA = 664;
    public double posB = -664;

    private double slotActual = SLOT_X;
    private double slotObjetivo = SLOT_X;

    private final PIDController pidLicuadora = new PIDController(0.01, 0.0, 0.0003);

    private double licuadoraTargetCounts = 0;

    private static final double toleranciaLicuadora = 8.0;

    // --- BLOQUEO LICUADORA  ---
    private static final long LEVANTADOR_MOVE_MS = 350;
    private long levantadorBusyUntilNanos = 0;

    private void setLevantador(double pos) {
        Levantador.setPosition(pos);
        levantadorBusyUntilNanos = System.nanoTime() + (LEVANTADOR_MOVE_MS * 1_000_000L);
    }

    private boolean levantadorBusy() {
        return System.nanoTime() < levantadorBusyUntilNanos;
    }
    // --------------------------------------------------------

    // --- BLOQUEO DE RUTINAS ---
    private static final int RUTINA_NINGUNA = 0;
    private static final int RUTINA_AUTO_PATH = 1;
    private static final int RUTINA_SHOOT_SINGLE = 2;
    private static final int RUTINA_SHOOT3 = 3;
    private static final int RUTINA_SHOOT3_PATRON = 4;

    private boolean rutinaEnCurso = false;
    private int rutinaActual = RUTINA_NINGUNA;

    private static final long cooldownRutinaMs = 250;
    private long rutinaCooldownUntilNanos = 0;

    private boolean iniciarRutina(int rutinaId) {
        long now = System.nanoTime();
        if (rutinaEnCurso) return false;
        if (now < rutinaCooldownUntilNanos) return false;

        rutinaEnCurso = true;
        rutinaActual = rutinaId;
        return true;
    }

    private void terminarRutina(int rutinaId) {
        if (rutinaEnCurso && rutinaActual == rutinaId) {
            rutinaEnCurso = false;
            rutinaActual = RUTINA_NINGUNA;
            rutinaCooldownUntilNanos = System.nanoTime() + (cooldownRutinaMs * 1_000_000L);
        }
    }
    // ------------------------------------------------------------

    // Toda la lógica del PID del lanzador
    private static final double Kp = 0.0045;
    private static final double Kd = 0.0004;
    private static final double Ki = 0.0015;

    boolean hayTag = false;
    double tx = 0.0;
    long stale = -1;
    int fidCount = 0;

    double correccion = 0.0;

    double currentAngle = 0.0;
    boolean tieneObjetivo = false;

    double errorAnterior = 0;
    double integral = 0;

    double referencia = 0;

    private long lastPidNanos = 0;
    private double txFilt = 0.0;

    private static final double TX_ALPHA = 0.25;
    private static final double DEADBAND_DEG = 0.1;

    private static final double SERVO_CENTER = 0.5;
    private static final double SERVO_RANGE = 0.25;
    private static final double I_MAX = 10.0;

    private double servoPosLanzador = SERVO_CENTER;

    private static final double RETURN_TO_CENTER_ALPHA = 0.06;

    boolean servosLanzadoresBusy = false;

    boolean botonLActual = false;
    boolean botonLAnterior = false;

    // --- Anti-spam licuadora ---
    private static final long manualLicuadoraCooldownMs = 160;
    private long manualLicuadoraCooldownUntilNanos = 0;
    // ---------------------------------

    private static final double NUDGE_STEP_COUNTS = 2.0;
    private static final double NUDGE_MAX_OFFSET_COUNTS = 120.0;
    private static final long NUDGE_PERIOD_MS = 35;
    private static final double NUDGE_TRIGGER_DEADBAND = 0.12;

    private double nudgeOffsetCounts = 0.0;
    private long lastNudgeNanos = 0;

    private static final int staleMaxFrames = 35;
    private static final long tagHoldMs = 250;
    private long ultimoTagBuenoNanos = 0;
    // =========================================

    // Toda la lógica de los followers (puntos pre programados)
    private Follower follower;

    private final Pose posicionTiroMedio = new Pose(72, 72, Math.toRadians(140));
    private final Pose posicionTiroAudiencia = new Pose(72
            , 24.83360258481423, Math.toRadians(120));
    private final Pose posicionSecretTunnel = new Pose(26.99030694668821
            , 64.12762520193863, Math.toRadians(180));
    private final Pose posicionHumanPlayer = new Pose(100.4361873990307
            , 20.180936995153473, Math.toRadians(0));
    private final Pose posicionAZonaDeParqueo = new Pose(90.7012987012987
            , 33.89610389610389, Math.toRadians(180));

    private boolean autoPath = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;

    private boolean PrevY = false;
    boolean prevY2 = false;
    boolean prevB = false;

    private void setLanzadorServos(double pos) {
        servoPosLanzador = Range.clip(pos, 0.0, 1.0);
        Lanzador_I.setPosition(servoPosLanzador);
        Lanzador_D.setPosition(servoPosLanzador);
        correccion = servoPosLanzador; // por telemetry
    }

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        motor_Iz_Fr = hardwareMap.get(DcMotorEx.class, "motor_Iz_Fr");
        motor_Iz_A = hardwareMap.get(DcMotorEx.class, "motor_Iz_A");
        motor_De_Fr = hardwareMap.get(DcMotorEx.class, "motor_Dr_Fr");
        motor_De_A = hardwareMap.get(DcMotorEx.class, "motor_Dr_A");

        motor_Iz_Fr.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_Iz_A.setDirection(DcMotorSimple.Direction.REVERSE);

        M_lanzador1 = hardwareMap.get(DcMotorEx.class, "M_lanzador1");

        M_lanzador1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        M_lanzador1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        M_lanzador2 = hardwareMap.get(DcMotorEx.class, "M_lanzador2");
        Recogedor = hardwareMap.get(DcMotorEx.class, "Recogedor");
        M_licuadora = hardwareMap.get(DcMotorEx.class, "licuadora");

        M_licuadora.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_licuadora.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        Lanzador_I = hardwareMap.get(Servo.class, "Lanzador_I");
        Lanzador_D = hardwareMap.get(Servo.class, "Lanzador_D");

        Direccion = hardwareMap.get(Servo.class, "Direccion");
        Levantador = hardwareMap.get(Servo.class, "Levantador");

        limaluz = hardwareMap.get(Limelight3A.class, "limaluz");
        limaluz.setPollRateHz(90);
        limaluz.pipelineSwitch(6);

        Direccion.setDirection(Servo.Direction.REVERSE);

        limaluz.start();

        Direccion.setPosition(0);

        // Servos de lanzamiento al centro
        setLanzadorServos(SERVO_CENTER);

        irASlot(SLOT_X);

        setLevantador(0.6);

        slotA = "Nada";
        slotB = "Nada";
        slotX = "Nada";

        inicioLanzador = 0;
        estadoLanzar = 0;
        estadoLanzador = false;

        M_lanzador1.setVelocityPIDFCoefficients(105, 0.00, 0.1, 12.0);
        M_lanzador2.setVelocityPIDFCoefficients(105, 0.00, 0.1, 12.0);

        configurePinpoint();

        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH,
                Storage.lista_PINPOINT[0],
                Storage.lista_PINPOINT[1],
                AngleUnit.DEGREES,
                Storage.lista_PINPOINT[2]
        ));
        pinpoint.update();

        if (Storage.valid) {
            lista_MOR.add(Storage.lista_MOR[0]);
            lista_MOR.add(Storage.lista_MOR[1]);
            lista_MOR.add(Storage.lista_MOR[2]);
        }

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

        long tInit = System.nanoTime();
        candidatoTimer = tInit;
        nadaTimer = tInit;
        ultimoGuardadoTimer = 0;

        waitForStart();

        while (opModeIsActive()) {

            licuadoraPID();

            // Pa que no le de una embolia

            boolean aAhora2 = gamepad2.a;
            boolean yAhora2 = gamepad2.y;
            boolean bAhora2 = gamepad2.b;

            boolean pressedY2 = yAhora2 && !prevY2;
            boolean pressedB2 = bAhora2 && !prevB;
            // ---------------------------------------------------------------

            // Movimiento del robot e implementación de los puntos de control (paths)

            boolean dpadRight2 = gamepad2.dpad_right;
            boolean dpadLeft2 = gamepad2.dpad_left;

            if (dpadRight2) {
                setLanzadorServos(servoPosLanzador + 0.01);
                servosLanzadoresBusy = true;
            } else if (dpadLeft2) {
                setLanzadorServos(servoPosLanzador - 0.01);
                servosLanzadoresBusy = true;
            } else {
                servosLanzadoresBusy = false;
            }

            boolean yPressed = gamepad1.y;

            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            boolean dpadRight = gamepad1.dpad_right;
            boolean dpadLeft = gamepad1.dpad_left;

            boolean pressedY = yPressed && !PrevY;
            boolean pressedLeft = dpadLeft && !prevDpadLeft;
            boolean pressedRight = dpadRight && !prevDpadRight;
            boolean pressedDown = dpadDown && !prevDpadDown;
            boolean pressedUp = dpadUp && !prevDpadUp;

            prevDpadLeft = dpadLeft;
            prevDpadRight = dpadRight;
            prevDpadDown = dpadDown;
            prevDpadUp = dpadUp;
            PrevY = yPressed;

            Storage.set1((float) pinpoint.getPosX(DistanceUnit.INCH),
                    (float) pinpoint.getPosY(DistanceUnit.INCH),
                    (float) pinpoint.getHeading(AngleUnit.DEGREES));

            if (gamepad1.a) {
                autoPath = false;
                follower.breakFollowing();
                terminarRutina(RUTINA_AUTO_PATH);
            }

            if (!autoPath && pressedUp && iniciarRutina(RUTINA_AUTO_PATH)) {
                AutoATiroMedio();
                autoPath = true;
            } else if (!autoPath && pressedDown && iniciarRutina(RUTINA_AUTO_PATH)) {
                AutoATiroAudiencia();
                autoPath = true;
            } else if (!autoPath && pressedRight && iniciarRutina(RUTINA_AUTO_PATH)) {
                AutoASecretTunnel();
                autoPath = true;
            } else if (!autoPath && pressedLeft && iniciarRutina(RUTINA_AUTO_PATH)) {
                AutoAHumanPlayer();
                autoPath = true;
            } else if (!autoPath && pressedY && iniciarRutina(RUTINA_AUTO_PATH)) {
                AutoAZonaDeParqueo();
                autoPath = true;
            }

            if (autoPath) {
                follower.update();
                if (!follower.isBusy()) {
                    autoPath = false;
                    terminarRutina(RUTINA_AUTO_PATH);
                }
            } else {
                double y = -gamepad1.left_stick_y;
                double rx = gamepad1.right_stick_x;
                double strafe = -gamepad1.left_trigger + gamepad1.right_trigger;

                double p1 = y + rx + strafe;
                double p2 = y + rx - strafe;
                double p3 = y - rx - strafe;
                double p4 = y - rx + strafe;

                motor_Iz_Fr.setPower(Range.clip(p1, -1.0, 1.0));
                motor_Iz_A.setPower(Range.clip(p2, -1.0, 1.0));
                motor_De_Fr.setPower(Range.clip(p3, -1.0, 1.0));
                motor_De_A.setPower(Range.clip(p4, -1.0, 1.0));
            }

            // Implementación de la lógica del cálculo de distancia y ángulo

            pinpoint.update();

            double pos_Y = pinpoint.getPosY(DistanceUnit.CM);
            double pos_X = pinpoint.getPosX(DistanceUnit.CM);

            double deltaX = goalX - pos_X;
            double deltaY = goalY - pos_Y;

            xb = (Math.sqrt(deltaX * deltaX + deltaY * deltaY)) / 100;

            Theta0 = Math.atan((2 * H2 / xb) - Math.tan(Math.toRadians(Thetai)));
            direccion = ((68 - Math.toDegrees(Theta0)) / 38) * 0.2;

            direccion = Range.clip(direccion, 0.0, 1.0);
            Direccion.setPosition(direccion);

            // Detección de color

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

            // Lógica del intake

            boolean intakeOn = gamepad1.left_bumper;
            boolean intakeOnInverso = gamepad1.right_bumper;

            boolean recolectandoAhora = intakeOn || intakeOnInverso;

            if (intakeOnInverso && !intakeOn) {
                Recogedor.setPower(-1);
            } else if (intakeOn) {
                Recogedor.setPower(1);

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

                    case recolectorEmpieza: {
                        if (guardadoPendiente && enPosicion()) {

                            boolean guardo = guardarEnSlotActual(guardadoPendienteColor);

                            guardadoPendiente = false;
                            guardadoPendienteColor = "Nada";

                            if (guardo) {
                                armadoNuevaPieza = false;
                                ultimoGuardadoTimer = tNow;

                                irASlot(siguienteSlotVacio(slotActual));

                                recolectTimer = tNow;
                                recolectState = recolectoCambiaLicuadora;
                            }
                        }
                        break;
                    }

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

            } else {
                Recogedor.setPower(0.0);
                recolectState = recolectorEmpieza;
                armadoNuevaPieza = true;

                guardadoPendiente = false;
                guardadoPendienteColor = "Nada";
            }

            // --- Movimiento manual licuadora (Right Bumper gamepad2) ---
            botonLActual = gamepad2.right_bumper;

            long nowManual = System.nanoTime();
            boolean manualEnCooldown = nowManual < manualLicuadoraCooldownUntilNanos;

            // Tinkilis
            boolean disparoEnEsteFrame = aAhora2 || yAhora2 || bAhora2 || pressedY2 || pressedB2;

            if (botonLActual && !botonLAnterior) {
                if (!manualEnCooldown
                        && !disparoEnEsteFrame
                        && !licuadoraBloqueadaPorEstado()
                        && enPosicion()) {

                    irASlot(siguienteSlot(slotObjetivo));
                    manualLicuadoraCooldownUntilNanos = nowManual + (manualLicuadoraCooldownMs * 1_000_000L);
                }
            }

            botonLAnterior = botonLActual;

            double lt2 = gamepad2.left_trigger;
            double rt2 = gamepad2.right_trigger;

            boolean nudgeLeft = lt2 > NUDGE_TRIGGER_DEADBAND;
            boolean nudgeRight = rt2 > NUDGE_TRIGGER_DEADBAND;

            if ((nudgeLeft || nudgeRight)
                    && !rutinaEnCurso
                    && !autoPath
                    && !disparoEnEsteFrame
                    && !manualEnCooldown
                    && !licuadoraBloqueadaPorEstado()) {

                long nowN = System.nanoTime();
                long ms = (lastNudgeNanos == 0) ? 9999 : (nowN - lastNudgeNanos) / 1_000_000;

                if (ms >= NUDGE_PERIOD_MS) {

                    double dir = 0.0;
                    if (nudgeRight && !nudgeLeft) dir = +1.0;
                    else if (nudgeLeft && !nudgeRight) dir = -1.0;

                    if (dir != 0.0) {
                        double mag = Math.max(lt2, rt2);
                        double step = NUDGE_STEP_COUNTS * (0.5 + 0.5 * mag);

                        nudgeOffsetCounts = Range.clip(
                                nudgeOffsetCounts + dir * step,
                                -NUDGE_MAX_OFFSET_COUNTS,
                                NUDGE_MAX_OFFSET_COUNTS
                        );

                        licuadoraTargetCounts = countsDeSlot(slotObjetivo) + nudgeOffsetCounts;

                        lastNudgeNanos = nowN;
                    }
                }
            }

            Storage.set1((float) pinpoint.getPosX(DistanceUnit.INCH),
                    (float) pinpoint.getPosY(DistanceUnit.INCH),
                    (float) pinpoint.getHeading(AngleUnit.DEGREES));

            // Disparo de un artefacto

            int v = (int) (852.02572 * Math.pow(1.16336, xb));

            M_lanzador1.setVelocity(v);
            M_lanzador2.setVelocity(v);

            if (aAhora2 && estadoLanzar == 0 && !shooting3 && !shooting3ConPatron
                    && !recolectandoAhora
                    && iniciarRutina(RUTINA_SHOOT_SINGLE)) {

                M_lanzador1.setVelocity(v);
                M_lanzador2.setVelocity(v);
                estadoLanzar = 3;
                estadoLanzador = true;
                inicioLanzador = System.nanoTime();

            } else if (estadoLanzar == 0 && !shooting3 && !shooting3ConPatron) {
                estadoLanzador = false;
            }

            if (estadoLanzador) {
                if (estadoLanzar == 3) {
                    double velocidadActual = M_lanzador1.getVelocity();
                    if (Math.abs(velocidadActual - v) < 30 && enPosicion()) {
                        estadoLanzar = 2;
                        setLevantador(1.0);
                        inicioLanzador = System.nanoTime();
                    }

                } else if (estadoLanzar == 2) {
                    if ((System.nanoTime() - inicioLanzador) / 1_000_000 > 550) {
                        setLevantador(0.6);
                        estadoLanzar = 0;
                        estadoLanzador = false;
                        limpiarSlotActual();
                        terminarRutina(RUTINA_SHOOT_SINGLE);
                    }

                } else {
                    if ((System.nanoTime() - inicioLanzador) / 1_000_000 > 3000) {
                        estadoLanzar = 0;
                        estadoLanzador = false;
                        terminarRutina(RUTINA_SHOOT_SINGLE);
                    }
                }
            } else {
                if (rutinaActual == RUTINA_SHOOT_SINGLE && estadoLanzar == 0) {
                    terminarRutina(RUTINA_SHOOT_SINGLE);
                }
            }

            if (pressedY2 && !shooting3 && !shooting3ConPatron
                    && !recolectandoAhora
                    && iniciarRutina(RUTINA_SHOOT3)) {

                shooting3 = true;
                shootState = disparadorEmpieza;
                disparosHechos = 0;
                bajadoLevantador = false;
            }

            if (shooting3) {
                if (disparar(v)) {
                    shooting3 = false;
                    resetDisparador();
                    terminarRutina(RUTINA_SHOOT3);
                }
            }

            prevY2 = yAhora2;

            if (pressedB2 && !shooting3ConPatron && !shooting3
                    && !recolectandoAhora
                    && iniciarRutina(RUTINA_SHOOT3_PATRON)) {

                calcularPosicion(0);
                shooting3ConPatron = true;
                shootState = disparadorEmpieza;
                disparosHechos = 0;
                bajadoLevantador = false;
            }

            if (shooting3ConPatron) {
                if (dispararEnPatron(v)) {
                    shooting3ConPatron = false;
                    resetDisparador();
                    terminarRutina(RUTINA_SHOOT3_PATRON);
                }
            }

            prevB = bAhora2;

            // Control PID (Limelight)
            updatePidCamara();

            Storage.set1((float) pinpoint.getPosX(DistanceUnit.INCH),
                    (float) pinpoint.getPosY(DistanceUnit.INCH),
                    (float) pinpoint.getHeading(AngleUnit.DEGREES));

            telemetry.addData("Detectado", detectado);
            telemetry.addData("slotX", slotX);
            telemetry.addData("slotA", slotA);
            telemetry.addData("slotB", slotB);
            telemetry.update();
        }

        if (portalColor != null) {
            portalColor.close();
        }

        M_licuadora.setPower(0);
    }

    // ---------------- PID CAMARA ----------------
    private void updatePidCamara() {

        long now = System.nanoTime();

        if (servosLanzadoresBusy) {
            integral = 0.0;
            errorAnterior = 0.0;
            lastPidNanos = now;
            return;
        }

        boolean tagFresco = false;

        LLResult res = limaluz.getLatestResult();
        if (res != null && res.isValid()) {

            List<LLResultTypes.FiducialResult> tags = res.getFiducialResults();
            int count = (tags == null) ? 0 : tags.size();

            long st = res.getStaleness(); // frames
            boolean stOk = (st >= 0 && st <= staleMaxFrames);

            tagFresco = (count > 0) && stOk;

            if (tagFresco) {
                double txMedido = res.getTx();

                txFilt = (ultimoTagBuenoNanos == 0) ? txMedido
                        : (TX_ALPHA * txMedido + (1.0 - TX_ALPHA) * txFilt);

                ultimoTagBuenoNanos = now;
            }
        }

        long msDesdeTag = (ultimoTagBuenoNanos == 0) ? 999999
                : (now - ultimoTagBuenoNanos) / 1_000_000;

        boolean tengoObjetivo = tagFresco || (msDesdeTag <= tagHoldMs);
        boolean usandoHold = tengoObjetivo && !tagFresco;

        if (!tengoObjetivo) {
            servoPosLanzador = servoPosLanzador + (SERVO_CENTER - servoPosLanzador) * RETURN_TO_CENTER_ALPHA;
            setLanzadorServos(servoPosLanzador);

            integral = 0.0;
            errorAnterior = 0.0;
            lastPidNanos = now;
            return;
        }

        double dt = (lastPidNanos == 0) ? 0.02 : (now - lastPidNanos) / 1e9;
        lastPidNanos = now;
        dt = Range.clip(dt, 0.005, 0.05);

        double error = referencia - txFilt;

        if (usandoHold) integral *= 0.92;

        if (Math.abs(error) < DEADBAND_DEG) {
            integral *= 0.85;
            errorAnterior = error;
            return;
        }

        if (!usandoHold) {
            integral += error * dt;
            integral = Range.clip(integral, -I_MAX, I_MAX);
        }

        double derivativa = 0.0;
        if (!usandoHold) derivativa = (error - errorAnterior) / dt;
        errorAnterior = error;

        double salida = Kp * error + Ki * integral + Kd * derivativa;
        salida = Range.clip(salida, -SERVO_RANGE, SERVO_RANGE);

        double targetPos = Range.clip(SERVO_CENTER + salida, 0.0, 1.0);

        double SMOOTH = 0.35;
        servoPosLanzador = servoPosLanzador + (targetPos - servoPosLanzador) * SMOOTH;

        if (servoPosLanzador <= 0.01 || servoPosLanzador >= 0.99) {
            integral *= 0.7;
        }

        setLanzadorServos(servoPosLanzador);
    }

    // -------------------------------------------------------

    private void resetDisparador() {
        shootState = disparadorEmpieza;
        bajadoLevantador = false;
        disparosHechos = 0;
        shootTimer = 0;
        slotTimer = 0;
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

    private boolean slotVacio(double slot) {
        if (slot == SLOT_X) return "Nada".equals(slotX);
        if (slot == SLOT_A) return "Nada".equals(slotA);
        return "Nada".equals(slotB);
    }

    private double siguienteSlotVacio(double base) {
        double s1 = siguienteSlot(base);
        if (slotVacio(s1)) return s1;
        double s2 = siguienteSlot(s1);
        if (slotVacio(s2)) return s2;
        double s3 = siguienteSlot(s2);
        if (slotVacio(s3)) return s3;
        return s1;
    }

    private void limpiarSlotActual() {
        if (slotActual == SLOT_X) slotX = "Nada";
        else if (slotActual == SLOT_A) slotA = "Nada";
        else slotB = "Nada";
    }

    private int siguienteSlot(double slot) {
        if (slot == SLOT_X) return SLOT_A;
        if (slot == SLOT_A) return SLOT_B;
        return SLOT_X;
    }

    private void irASlot(double slot) {
        slotObjetivo = slot;
        licuadoraTargetCounts = countsDeSlot(slot) + nudgeOffsetCounts;
    }

    private double countsDeSlot(double slot) {
        if (slot == SLOT_A) return posA;
        if (slot == SLOT_B) return posB;
        return posX;
    }

    private boolean enPosicion() {
        double err = licuadoraTargetCounts - M_licuadora.getCurrentPosition();
        return Math.abs(err) <= toleranciaLicuadora;
    }

    private void licuadoraPID() {

        // Si el levantar está moviendos, la licuador NO se mueve. PUNTO.
        // LICUADORA MALPARIDA

        if (levantadorBusy()) {
            M_licuadora.setPower(0);
            pidLicuadora.reset();
            return;
        }

        int current = M_licuadora.getCurrentPosition();
        double error = licuadoraTargetCounts - current;

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

    private boolean disparar(int v) {
        switch (shootState) {

            case disparadorEmpieza:
                M_lanzador1.setVelocity(v);
                M_lanzador2.setVelocity(v);
                shootTimer = System.nanoTime();
                shootState = disparadorSube;
                return false;

            case disparadorSube: {
                double velActual = (M_lanzador1.getVelocity() + M_lanzador2.getVelocity()) / 2;
                long ms = (System.nanoTime() - shootTimer) / 1_000_000;

                if ((Math.abs(velActual - v) < 200 || ms > 500) && enPosicion() && !levantadorBusy()) {
                    setLevantador(1.0);
                    shootTimer = System.nanoTime();
                    shootState = disparadorBaja;
                    bajadoLevantador = false;
                }
                return false;
            }

            case disparadorBaja: {
                long ms = (System.nanoTime() - shootTimer) / 1_000_000;

                if (!bajadoLevantador && ms > bajarMs) {
                    setLevantador(0.6);
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

                    irASlot(siguienteSlot(slotActual));

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
                shootState = disparadorEmpieza;
                return true;
        }
    }

    private void calcularPosicion(int pos) {
        if (lista_MOR.size() <= pos) return;

        boolean wantPurpura = lista_MOR.get(pos);
        String objetivo = wantPurpura ? "Purpura" : "Verde";

        int slotTarget = -1;
        if (objetivo.equals(slotX)) slotTarget = SLOT_X;
        else if (objetivo.equals(slotA)) slotTarget = SLOT_A;
        else if (objetivo.equals(slotB)) slotTarget = SLOT_B;

        if (slotTarget == -1) {
            disparosHechos++;
            return;
        }

        irASlot(slotTarget);
    }

    public boolean dispararEnPatron(int v) {

        switch (shootState) {

            case disparadorEmpieza:
                M_lanzador1.setVelocity(v);
                M_lanzador2.setVelocity(v);
                shootTimer = System.nanoTime();
                shootState = disparadorSube;
                return false;

            case disparadorSube: {
                double velActual = (M_lanzador1.getVelocity() + M_lanzador2.getVelocity()) / 2;
                long ms = (System.nanoTime() - shootTimer) / 1_000_000;

                if ((Math.abs(velActual - v) < 200 || ms > 500) && enPosicion() && !levantadorBusy()) {
                    setLevantador(1.0);
                    shootTimer = System.nanoTime();
                    shootState = disparadorBaja;
                    bajadoLevantador = false;
                }
                return false;
            }

            case disparadorBaja: {
                long ms = (System.nanoTime() - shootTimer) / 1_000_000;

                if (!bajadoLevantador && ms > bajarMs) {
                    setLevantador(0.6);
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
                shootState = disparadorEmpieza;
                return true;
        }
    }

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

    private boolean licuadoraBloqueadaPorEstado() {
        boolean rutinaBloquea = rutinaEnCurso && (
                rutinaActual == RUTINA_SHOOT_SINGLE
                        || rutinaActual == RUTINA_SHOOT3
                        || rutinaActual == RUTINA_SHOOT3_PATRON
        );

        return levantadorBusy()
                || rutinaBloquea
                || estadoLanzador
                || (estadoLanzar != 0)
                || shooting3
                || shooting3ConPatron
                || (shootState != disparadorEmpieza)
                || (recolectState != recolectorEmpieza);
    }

    public void configurePinpoint() {
        pinpoint.setOffsets(-110 / 2.54, -160 / 2.54, DistanceUnit.MM);

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
    }

    private void AutoATiroAudiencia() {
        pinpoint.update();

        motor_Iz_Fr.setPower(0);
        motor_Iz_A.setPower(0);
        motor_De_Fr.setPower(0);
        motor_De_A.setPower(0);

        Pose start = new Pose(
                pinpoint.getPosX(DistanceUnit.INCH),
                pinpoint.getPosY(DistanceUnit.INCH),
                pinpoint.getHeading(AngleUnit.RADIANS)
        );

        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(start, posicionTiroAudiencia))
                .setLinearHeadingInterpolation(start.getHeading(), posicionTiroAudiencia.getHeading())
                .build();

        follower.followPath(path, true);
    }

    private void AutoASecretTunnel() {
        pinpoint.update();

        motor_Iz_Fr.setPower(0);
        motor_Iz_A.setPower(0);
        motor_De_Fr.setPower(0);
        motor_De_A.setPower(0);

        Pose start = new Pose(
                pinpoint.getPosX(DistanceUnit.INCH),
                pinpoint.getPosY(DistanceUnit.INCH),
                pinpoint.getHeading(AngleUnit.RADIANS)
        );

        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(start, posicionSecretTunnel))
                .setLinearHeadingInterpolation(start.getHeading(), posicionSecretTunnel.getHeading())
                .build();

        follower.followPath(path, true);
    }

    private void AutoAHumanPlayer() {
        pinpoint.update();

        motor_Iz_Fr.setPower(0);
        motor_Iz_A.setPower(0);
        motor_De_Fr.setPower(0);
        motor_De_A.setPower(0);

        Pose start = new Pose(
                pinpoint.getPosX(DistanceUnit.INCH),
                pinpoint.getPosY(DistanceUnit.INCH),
                pinpoint.getHeading(AngleUnit.RADIANS)
        );

        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(start, posicionHumanPlayer))
                .setLinearHeadingInterpolation(start.getHeading(), posicionHumanPlayer.getHeading())
                .build();

        follower.followPath(path, true);
    }

    private void AutoAZonaDeParqueo() {
        pinpoint.update();

        motor_De_A.setPower(0);
        motor_De_Fr.setPower(0);
        motor_Iz_A.setPower(0);
        motor_Iz_Fr.setPower(0);

        Pose start = new Pose(
                pinpoint.getPosX(DistanceUnit.INCH),
                pinpoint.getPosY(DistanceUnit.INCH),
                pinpoint.getHeading(AngleUnit.RADIANS)
        );

        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(start, posicionAZonaDeParqueo))
                .setLinearHeadingInterpolation(start.getHeading(), posicionAZonaDeParqueo.getHeading())
                .build();

        follower.followPath(path, true);
    }

    private void AutoATiroMedio() {
        pinpoint.update();

        motor_Iz_Fr.setPower(0);
        motor_Iz_A.setPower(0);
        motor_De_Fr.setPower(0);
        motor_De_A.setPower(0);

        Pose start = new Pose(
                pinpoint.getPosX(DistanceUnit.INCH),
                pinpoint.getPosY(DistanceUnit.INCH),
                pinpoint.getHeading(AngleUnit.RADIANS)
        );

        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(start, posicionTiroMedio))
                .setLinearHeadingInterpolation(start.getHeading(), posicionTiroMedio.getHeading())
                .build();

        follower.followPath(path, true);
    }
}