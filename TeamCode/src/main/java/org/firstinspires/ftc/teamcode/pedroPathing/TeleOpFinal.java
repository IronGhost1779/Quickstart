package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.robotcore.util.Range;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

@TeleOp
public class TeleOpFinal extends LinearOpMode {

    GoBildaPinpointDriver pinpoint;

    private DcMotorEx motor_Iz_Fr, motor_Iz_A, motor_De_Fr, motor_De_A;

    private DcMotorEx M_lanzador1, M_lanzador2;

    private Servo Lanzador_I, Lanzador_D, Levantador, Direccion, Licuadora;

    private TouchSensor sensorM;

    private Limelight3A limaluz;

    private static final int NUM_DISPAROS = 3;
    private int disparosHechos = 0;
    private long shootTimer = 0;

    private boolean bajadoLevantador = false;

    private static final long bajarMs = 500;
    private static final long cambiarLicuadoraMs = 450;
    private static final long esperarAntesDeSubirMs = 1100;

    private long slotTimer = 0;

    // Disparador
    private static final int disparadorEmpieza = 0;
    private static final int disparadorSube = 1;
    private static final int disparadorBaja = 2;
    private static final int disparadorFin = 3;
    private static final int disparadorEsperarSlot = 4;
    private int shootState = disparadorEmpieza;

    // Recoleccion

    private static final int recolectorEmpieza = 0;

    boolean shooting3 = false;
    boolean prevY = false;

    double goalY = 332.74; //cm
    double goalX = 40.64; //cm

    double H2 = 0.85675;

    double Thetai = -40;
    double Theta0 = 0;

    double V0 = 0;

    double xb;

    double g = 9.77;

    double direccion = 0;
    double velM;

    // Estos dos los calculé (son medidas fisicas)
    double radioLanzador = 0.05;
    double ticksPorRevolucionMotor = 28;

    String detectado = "Nada";

    enum Modo { A, X, B }

    String slotA = "Nada";
    String slotB = "Nada";
    String slotX = "Nada";

    double posA = 0.473;
    double posB = 0.95;
    double posX = 0.0;

    double Kp = 0.002;
    double Ki = 0.0000;
    double Kd = 0.002;

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
    private static final double DEADBAND_DEG = 0.255;
    private static final double SERVO_CENTER = 0.5;
    private static final double SERVO_RANGE = 0.25;
    private static final double I_MAX = 10.0;

    boolean servosLanzadoresBusy = false;

    private String[] data = {"", "", ""};

    Modo modo = Modo.A;

    boolean botonL_actual = false;
    boolean botonL_anterior = false;

    int contador = 0;

    int estado_lanzar = 0;
    boolean estado_lanzador = false;

    long inicio_lanzador = 0;

    boolean lastPiecePresent = false;

    private Follower follower;

    private final Pose posicionTiroMedio = new Pose(72, 72, Math.toRadians(140));
    private final Pose posicionTiroAudiencia = new Pose(72, 24.83360258481423, Math.toRadians(120));
    private final Pose posicionSecretTunnel = new Pose(26.99030694668821, 71.12762520193863, Math.toRadians(180));
    private final Pose posicionHumanPlayer = new Pose(110.4361873990307, 20.180936995153473, Math.toRadians(0));

    private boolean autoPath = false;

    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;

    private PredominantColorProcessor sensorColor;
    private VisionPortal portalColor;

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        sensorM = hardwareMap.get(TouchSensor.class, "Magnetico");

        motor_Iz_Fr = hardwareMap.get(DcMotorEx.class, "motor_Iz_Fr");
        motor_Iz_A  = hardwareMap.get(DcMotorEx.class, "motor_Iz_A");
        motor_De_Fr = hardwareMap.get(DcMotorEx.class, "motor_Dr_Fr");
        motor_De_A  = hardwareMap.get(DcMotorEx.class, "motor_Dr_A");

        motor_Iz_Fr.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_Iz_A.setDirection(DcMotorSimple.Direction.REVERSE);

        M_lanzador1 = hardwareMap.get(DcMotorEx.class, "M_lanzador1");
        M_lanzador2 = hardwareMap.get(DcMotorEx.class, "M_lanzador2");

        M_lanzador1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        M_lanzador1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        Lanzador_I  = hardwareMap.get(Servo.class, "Lanzador_I");
        Lanzador_D  = hardwareMap.get(Servo.class, "Lanzador_D");

        Direccion   = hardwareMap.get(Servo.class, "Direccion");
        Licuadora   = hardwareMap.get(Servo.class, "Licuadora");
        Levantador  = hardwareMap.get(Servo.class, "Levantador");

        limaluz = hardwareMap.get(Limelight3A.class, "limaluz");
        limaluz.setPollRateHz(90);
        limaluz.pipelineSwitch(6);

        Direccion.setDirection(Servo.Direction.REVERSE);

        limaluz.start();

        Direccion.setPosition(0);

        Licuadora.setPosition(posX);
        modo = Modo.X;

        slotB = data[0];
        slotA = data[1];
        slotX = data[2];

        Levantador.setPosition(0.6);

        slotA = "Nada";
        slotB = "Nada";
        slotX = "Nada";

        contador = 0;
        inicio_lanzador = 0;
        estado_lanzar = 0;
        estado_lanzador = false;

        M_lanzador1.setVelocityPIDFCoefficients(105, 0.0, 0.1, 12.0);

        configurePinpoint();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 56, 8, AngleUnit.DEGREES, 90));
        pinpoint.update();

        // Cámara

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


        waitForStart();

        while (opModeIsActive()) {

            // ---------------- Movimiento lanzador -----------------

            boolean dpadRight2 = gamepad2.dpad_right;
            boolean dpadLeft2 = gamepad2.dpad_left;

            if(dpadRight2){
                Lanzador_D.setPosition(Lanzador_D.getPosition() - 0.01);
                Lanzador_I.setPosition(Lanzador_D.getPosition() + 0.01);

                servosLanzadoresBusy = true;
            }else if(dpadLeft2){
                Lanzador_D.setPosition(Lanzador_D.getPosition() + 0.01);
                Lanzador_I.setPosition(Lanzador_D.getPosition() - 0.01);

                servosLanzadoresBusy = true;
            }else{
                servosLanzadoresBusy = false;
            }

            // ----------------- Follower del path ------------------

            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            boolean dpadRight = gamepad1.dpad_right;
            boolean dpadLeft = gamepad1.dpad_left;

            boolean pressedLeft = dpadLeft && !prevDpadLeft;
            boolean pressedRight = dpadRight && !prevDpadRight;
            boolean pressedDown = dpadDown && !prevDpadDown;
            boolean pressedUp = dpadUp && !prevDpadUp;

            prevDpadLeft = dpadLeft;
            prevDpadRight = dpadRight;
            prevDpadDown = dpadDown;
            prevDpadUp = dpadUp;

            if(gamepad1.a){
                autoPath = false;
                follower.breakFollowing();
            }

            if (!autoPath && pressedUp) {
                AutoATiroMedio();
                autoPath = true;
            }else if(!autoPath && pressedDown){
                AutoATiroAudiencia();
                autoPath = true;
            }else if(!autoPath && pressedLeft){
                AutoASecretTunnel();
                autoPath = true;
            }else if(!autoPath && pressedRight){
                AutoAHumanPlayer();
                autoPath = true;
            }

            if (autoPath) {
                follower.update();
                if (!follower.isBusy()) {
                    autoPath = false;
                }
            } else {

                // ------------------------- Drive ---------------------------------
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

            // ---------------------- Calculos lanzador ------------------------

            pinpoint.update();

            double pos_Y = pinpoint.getPosY(DistanceUnit.CM);
            double pos_X = pinpoint.getPosX(DistanceUnit.CM);

            double deltaX = goalX - pos_X;
            double deltaY = goalY - pos_Y;

            xb = (Math.sqrt(deltaX*deltaX + deltaY*deltaY))/100;

            Theta0 = Math.atan((2*H2/xb) - Math.tan(Math.toRadians(Thetai)));

            V0 = ((1/Math.cos(Theta0))*Math.sqrt(g*xb/(Math.tan(Theta0)-Math.tan(Math.toRadians(Thetai)))));

            velM = (V0/(Math.PI*radioLanzador))*(ticksPorRevolucionMotor);
            direccion = ((68 - Math.toDegrees(Theta0))/38)*0.2;

            Direccion.setPosition(direccion);

            telemetry.addData("dist", xb);
            telemetry.addData("X: ", pos_X);
            telemetry.addData("Y: ", pos_Y);

            // ---------------------------- Recogedor --------------------------

            boolean piecePresent = !"Nada".equals(detectado);

            if (gamepad1.left_bumper) {
                M_lanzador2.setPower(1.0);

                if (piecePresent && !lastPiecePresent && contador < 3) {

                    if (modo == Modo.X && "Nada".equals(slotX)) {
                        slotX = detectado;

                        Licuadora.setPosition(posA);
                        modo = Modo.A;
                        contador++;

                        detectado = "Nada";

                    } else if (modo == Modo.A && "Nada".equals(slotA)) {
                        slotA = detectado;

                        Licuadora.setPosition(posB);
                        modo = Modo.B;
                        contador++;

                        detectado = "Nada";

                    } else if (modo == Modo.B && "Nada".equals(slotB)) {
                        slotB = detectado;

                        Licuadora.setPosition(posX);
                        modo = Modo.X;
                        contador++;

                        detectado = "Nada";
                    }
                }

                lastPiecePresent = piecePresent;

            } else {
                M_lanzador2.setPower(0.0);
                lastPiecePresent = false;
            }

            // ---------------- Deteccion con cámara ----------------
            detectado = "Nada";

            if (portalColor != null && sensorColor != null) {
                PredominantColorProcessor.Result resultColor = sensorColor.getAnalysis();

                if (resultColor != null) {
                    if (resultColor.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN) {
                        detectado = "Verde";
                    } else if (resultColor.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) {
                        detectado = "Purpura";
                    }
                    telemetry.addData("SwatchCam", resultColor.closestSwatch);
                    telemetry.addLine(String.format("RGB (%3d,%3d,%3d)",
                            resultColor.RGB[0], resultColor.RGB[1], resultColor.RGB[2]));
                }
            } else {
                telemetry.addLine("No detecta");
            }

            // ---------------- Cambio manual de licuadora ----------------
            botonL_actual = gamepad2.right_bumper;
            if (botonL_actual && !botonL_anterior) {
                if (modo == Modo.X) {
                    Licuadora.setPosition(posA);
                    modo = Modo.A;
                } else if (modo == Modo.A) {
                    Licuadora.setPosition(posB);
                    modo = Modo.B;
                } else {
                    Licuadora.setPosition(posX);
                    modo = Modo.X;
                }
            }
            botonL_anterior = botonL_actual;

            // ------------------- Lanzador inteligente ------------------------

            int v = (int)(1436.44779*Math.pow(1.08097, xb));

            if(this.gamepad2.a && estado_lanzar == 0){
                M_lanzador1.setVelocity(v);
                estado_lanzar = 3;
                estado_lanzador = true;
                inicio_lanzador = System.nanoTime();
            }else if(estado_lanzar == 0 && !shooting3){
                estado_lanzador = false;
                M_lanzador1.setVelocity(0);
            }

            if (estado_lanzador) {
                if (estado_lanzar == 3) {
                    double velocidadActual = M_lanzador1.getVelocity();
                    telemetry.addData("Vi velocidad Actual es", velocidadActual);
                    if (Math.abs(velocidadActual - v) < 30){
                        estado_lanzar = 2;
                        Levantador.setPosition(1.0);
                        inicio_lanzador = System.nanoTime();
                    }

                } else if (estado_lanzar == 2) {
                    if ((System.nanoTime() - inicio_lanzador) / 1_000_000 > 550) {
                        Levantador.setPosition(0.6);
                        estado_lanzar = 0;
                        estado_lanzador = false;

                        if (modo == Modo.X) slotX = "Nada";
                        else if (modo == Modo.A) slotA = "Nada";
                        else slotB = "Nada";
                    }

                } else if (estado_lanzar == 1) {
                    if ((System.nanoTime() - inicio_lanzador) / 1_000_000 > 2000) {
                        inicio_lanzador = System.nanoTime();
                        estado_lanzar = 2;
                        Levantador.setPosition(1.0);
                    }

                } else {
                    if ((System.nanoTime() - inicio_lanzador) / 1_000_000 > 3000) {
                        estado_lanzador = false;
                        M_lanzador1.setVelocity(0);
                    }
                }
            }

            // ---------------------- DISPARADOR DE 3 ----------------------

            boolean yAhora = gamepad2.y;

            if (yAhora && !prevY && !shooting3) {
                Licuadora.setPosition(posX);
                modo = Modo.X;
                shooting3 = true;
                shootState = disparadorEmpieza;
                disparosHechos = 0;
                bajadoLevantador = false;
            }

            if (shooting3) {
                if (disparar(v)) {
                    shooting3 = false;
                }
            }

            prevY = yAhora;

            // ------------------------ PID AprilTag ---------------------------
            LLResult res = limaluz.getLatestResult();

            hayTag = false;
            fidCount = 0;
            stale = -1;

            if (res != null && res.isValid()) {
                stale = res.getStaleness();

                List<LLResultTypes.FiducialResult> tags = res.getFiducialResults();

                fidCount = (tags == null) ? 0 : tags.size();
                hayTag = fidCount > 0;

                if (hayTag) {
                    tx = res.getTx();
                }

                if (stale > 10) hayTag = false;
            }

            tieneObjetivo = hayTag;

            long now = System.nanoTime();
            double dt = (lastPidNanos == 0) ? 0.02 : (now - lastPidNanos) / 1e9;
            lastPidNanos = now;
            dt = Range.clip(dt, 0.005, 0.05);

            if (tieneObjetivo) {
                txFilt = TX_ALPHA * tx + (1.0 - TX_ALPHA) * txFilt;
            }

            if (sensorM.isPressed()) {
                centrarLanzadorServos();

            }else if(!servosLanzadoresBusy){

                if (!tieneObjetivo) {
                    centrarLanzadorServos();
                } else {
                    currentAngle = txFilt;

                    double error = referencia - currentAngle;

                    if (Math.abs(error) < DEADBAND_DEG) {
                        centrarLanzadorServos();

                    } else {

                        integral += error * dt;
                        integral = Range.clip(integral, -I_MAX, I_MAX);

                        double derivativa = (error - errorAnterior) / dt;

                        double salida = Kp * error + Ki * integral + Kd * derivativa;
                        salida = Range.clip(salida, -SERVO_RANGE, SERVO_RANGE);

                        correccion = SERVO_CENTER + salida;
                        correccion = Range.clip(correccion, 0.0, 1.0);

                        if (correccion <= 0.01 || correccion >= 0.99) {
                            integral *= 0.7;
                        }

                        Lanzador_I.setPosition(correccion);
                        Lanzador_D.setPosition(correccion);
                    }

                    errorAnterior = error;
                }
            }

            // ---------------- Lanzador inteligente ----------------
            boolean pedirPurpura = gamepad2.left_stick_button;
            boolean pedirVerde   = gamepad2.right_stick_button;

            if (!estado_lanzador && (pedirPurpura || pedirVerde)) {

                String objetivo = pedirPurpura ? "Purpura" : "Verde";

                boolean existe = objetivo.equals(slotX) || objetivo.equals(slotA) || objetivo.equals(slotB);

                if (!existe) {
                    estado_lanzador = false;
                    estado_lanzar = 0;
                } else {
                    estado_lanzador = true;
                    M_lanzador1.setVelocity(v);

                    if ((modo == Modo.X && objetivo.equals(slotX)) ||
                            (modo == Modo.A && objetivo.equals(slotA)) ||
                            (modo == Modo.B && objetivo.equals(slotB))) {

                        estado_lanzar = 3;
                        estado_lanzador = true;
                        inicio_lanzador = System.nanoTime();

                    } else {
                        if (modo == Modo.X) {
                            if (objetivo.equals(slotA)) {
                                Licuadora.setPosition(posA);
                                modo = Modo.A;
                            } else {
                                Licuadora.setPosition(posB);
                                modo = Modo.B;
                            }
                        } else if (modo == Modo.A) {
                            if (objetivo.equals(slotX)) {
                                Licuadora.setPosition(posX);
                                modo = Modo.X;
                            } else {
                                Licuadora.setPosition(posB);
                                modo = Modo.B;
                            }
                        } else {
                            if (objetivo.equals(slotA)) {
                                Licuadora.setPosition(posA);
                                modo = Modo.A;
                            } else {
                                Licuadora.setPosition(posX);
                                modo = Modo.X;
                            }
                        }

                        estado_lanzar = 3;
                        estado_lanzador = true;
                        inicio_lanzador = System.nanoTime();
                    }
                }
            }

            // ---------------- Telemetry ----------------
            telemetry.addData("movimeinto de velocidad", M_lanzador1.isBusy());
            telemetry.addData("Detectado", detectado);
            telemetry.addData("slotX", slotX);
            telemetry.addData("slotA", slotA);
            telemetry.addData("slotB", slotB);
            telemetry.update();
        }

        if (portalColor != null) {
            portalColor.close();
        }
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

    private void centrarLanzadorServos() {
        Lanzador_I.setPosition(0.5);
        Lanzador_D.setPosition(0.5);
        correccion = 0.5;
        integral = 0.0;
        errorAnterior = 0.0;
    }

    private boolean disparar(int v) {

        switch (shootState) {
            case disparadorEmpieza:
                M_lanzador1.setVelocity(v);
                shootTimer = System.nanoTime();
                shootState = disparadorSube;
                return false;

            case disparadorSube: {
                double velActual = M_lanzador1.getVelocity();
                long ms = (System.nanoTime() - shootTimer) / 1_000_000;

                if (Math.abs(velActual - v) < 30 || ms > 500) {
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
                    disparosHechos++;
                    bajadoLevantador = true;
                    shootTimer = System.nanoTime();
                    return false;
                }

                if (bajadoLevantador && ms > cambiarLicuadoraMs) {

                    if (disparosHechos == 1) {
                        Licuadora.setPosition(posB);
                        modo = Modo.B;
                    } else if (disparosHechos == 2) {
                        Licuadora.setPosition(posA);
                        modo = Modo.A;
                    } else {
                        Licuadora.setPosition(posX);
                        modo = Modo.X;
                    }

                    if (disparosHechos >= NUM_DISPAROS) {
                        M_lanzador1.setVelocity(0);
                        shootState = disparadorFin;
                        return true;
                    }

                    slotTimer = System.nanoTime();
                    shootState = disparadorEsperarSlot;
                    return false;
                }

                return false;
            }

            case disparadorEsperarSlot:
                long ms = (System.nanoTime() - slotTimer) / 1_000_000;
                if (ms > esperarAntesDeSubirMs) {
                    shootState = disparadorEmpieza;
                }
                return false;

            case disparadorFin:
            default:
                return true;
        }
    }

    public void configurePinpoint(){

        pinpoint.setOffsets(-110/2.54, -160/2.54, DistanceUnit.MM);

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

    }
}