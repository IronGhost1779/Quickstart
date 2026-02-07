package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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
import android.util.Size;

@Autonomous(name="Auto Azul Audiencia")
public class AutoAzulAudiencia extends OpMode {

    private DcMotorEx M_lanzador1, M_lanzador2;

    private Servo Direccion, Levantador, Licuadora;
    private Servo Lanzador_I, Lanzador_D;

    private Limelight3A limaluz;
    private GoBildaPinpointDriver pinpoint;

    String detectado = "Nada";

    enum Modo { A, X, B }

    String slotA = "Nada";
    String slotB = "Nada";
    String slotX = "Nada";

    Modo modo = Modo.X;

    int contador = 0;
    private PredominantColorProcessor sensorColor;
    private VisionPortal portalColor;
    boolean lastPiecePresent = false;
    public int leidoID = -1;

    ArrayList<Boolean> lista_MOR = new ArrayList<Boolean>();

    private static final int NUM_DISPAROS = 3;
    private int disparosHechos = 0;
    private long shootTimer = 0;

    double posA = 0.473;
    double posB = 0.95;
    double posX = 0.0;

    double goalY = 332.74; // cm
    double goalX = 40.64;  // cm
    double xb;

    double H2 = 0.85675;
    double Thetai = -40;
    double Theta0 = 0;
    double direccion = 0;

    // ---------------- CONTROL ----------------
    private static final double kP = 0.005;

    private static final double kD = 0.0015;

    private double errorAnterior = 0.0;
    private long pidTimer = 0;

    private static final double tolerancia = 0.255;
    private static final double SERVO_CENTER = 0.5;
    private static final double SERVO_RANGE = 0.25;

    private double correccion = SERVO_CENTER;

    // ---------------- PATH STATES ----------------
    private static final int INICIO_A_RECOLECCION = 0;
    private static final int RECOLECCION = 1;
    private static final int RECOLECCION1_A_LANZAMIENTO1 = 2;
    private static final int LANZAMIENTO1_A_RECOLECCION2 = 3;
    private static final int RECOLECCION2 = 4;
    private static final int RECOLECCION2_A_LANZAMIENTO2 = 5;
    private static final int LANZAMIENTO2_A_RECOLECCION3 = 6;
    private static final int RECOLECCION3 = 7;
    private static final int RECOLECCION3_A_LANZAMIENTO3 = 8;
    private static final int FIN = 9;
    private static final int DONE = 10;

    // ---------------- PRE-ESTADOS ----------------
    private static final int preLeerTagObelisco = 0;
    private static final int preAlinear = 1;
    private static final int preDisparar = 2;
    private static final int preListo = 3;
    private int preEstado = preLeerTagObelisco;

    private static final long p5Ms = 80;

    private boolean bajadoLevantador = false;

    private static final long bajarMs = 350;
    private static final long cambiarLicuadoraMs = 500;
    private static final long timeutAlinearMs = 800;
    private static final long esperarAntesDeSubirMs = 1000;

    private long slotTimer = 0;
    private long preStart = 0;
    private long alignStart = 0;

    // ---------------- DISPARO ----------------
    private static final int disparadorEmpieza = 0;
    private static final int disparadorSube = 1;
    private static final int disparadorBaja = 2;
    private static final int disparadorFin = 3;
    private static final int disparadorEsperarSlot = 4;
    private int shootState = disparadorEmpieza;

    private Follower follower;
    private Timer pathTimer;

    private int estado = INICIO_A_RECOLECCION;

    // ---------------- POSES / PATHS ----------------

    // inicioARecoleccion1
    private final Pose posicionInicio = new Pose(56, 8, Math.toRadians(90));
    private final Pose control1 = new Pose(55.324717285945084, 29.25848142164781);
    private final Pose posicionRecoleccion1 = new Pose(42.50726978998386, 35.76736672051696, Math.toRadians(180));

    // recoleccion1
    private final Pose posicionFinRecoleccion1 = new Pose(24.827140549273018, 35.76736672051696);

    // recoleccion1Atiro1
    private final Pose posicionRecoleccion1Atiro1 = new Pose(46.833602584814216, 15.558966074313398, Math.toRadians(90));

    // tiro1ARecoleccion2
    private final Pose posicionTiro1ARecoleccion2 = new Pose(42.68336025848143, 60.26009693053314, Math.toRadians(180));

    // recoleccion2
    private final Pose posicionFinRecoleccion2 = new Pose(24.02746365105009, 59.93053311793216, Math.toRadians(180));

    // recoleccion2Atiro2
    private final Pose posicionRecoleccion2Atiro2 = new Pose(61.424878836833614, 81.37964458804524, Math.toRadians(125));

    // tiro2ARecoleccion3
    private final Pose posicionTiro2ARecoleccion3 = new Pose(41.903069466882066, 84.10339256865913, Math.toRadians(180));
    private final Pose control2 = new Pose(51.035541195476576, 81.67043618739903);

    // recoleccion3
    private final Pose posicionFinRecoleccion3 = new Pose(27.553150242326338, 83.92245557350566, Math.toRadians(180));

    // recoleccion3Atiro3
    private final Pose posicionRecoleccion3Atiro3 = new Pose(43.953150242326316, 94.41680129240711, Math.toRadians(128));

    // fin
    private final Pose posicionFin = new Pose(24.169628432956372, 70.24555735056542, Math.toRadians(180));
    private final Pose control3 = new Pose(43.89095315024232, 70.88691437802909);

    private PathChain inicioARecoleccion1, recoleccion1, recoleccion1Atiro1, tiro1ARecoleccion2, recoleccion2,
            recoleccion2Atiro2, tiro2ARecoleccion3, recoleccion3, recoleccion3Atiro3, fin;

    public void buildPaths() {
        inicioARecoleccion1 = follower.pathBuilder()
                .addPath(new BezierCurve(posicionInicio, control1, posicionRecoleccion1))
                .setLinearHeadingInterpolation(posicionInicio.getHeading(), posicionRecoleccion1.getHeading())
                .build();

        recoleccion1 = follower.pathBuilder()
                .addPath(new BezierLine(posicionRecoleccion1, posicionFinRecoleccion1))
                .setLinearHeadingInterpolation(posicionRecoleccion1.getHeading(), posicionRecoleccion1.getHeading())
                .build();

        recoleccion1Atiro1 = follower.pathBuilder()
                .addPath(new BezierLine(posicionFinRecoleccion1, posicionRecoleccion1Atiro1))
                .setLinearHeadingInterpolation(posicionRecoleccion1.getHeading(),
                        posicionRecoleccion1Atiro1.getHeading())
                .build();

        tiro1ARecoleccion2 = follower.pathBuilder()
                .addPath(new BezierLine(posicionRecoleccion1Atiro1, posicionTiro1ARecoleccion2))
                .setLinearHeadingInterpolation(posicionRecoleccion1Atiro1.getHeading(),
                        posicionTiro1ARecoleccion2.getHeading())
                .build();

        recoleccion2 = follower.pathBuilder()
                .addPath(new BezierLine(posicionTiro1ARecoleccion2, posicionFinRecoleccion2))
                .setLinearHeadingInterpolation(posicionTiro1ARecoleccion2.getHeading(),
                        posicionFinRecoleccion2.getHeading())
                .build();

        recoleccion2Atiro2 = follower.pathBuilder()
                .addPath(new BezierLine(posicionFinRecoleccion2, posicionRecoleccion2Atiro2))
                .setLinearHeadingInterpolation(posicionFinRecoleccion2.getHeading(),
                        posicionRecoleccion2Atiro2.getHeading())
                .build();

        tiro2ARecoleccion3 = follower.pathBuilder()
                .addPath(new BezierCurve(posicionRecoleccion2Atiro2, control2, posicionTiro2ARecoleccion3))
                .setLinearHeadingInterpolation(posicionRecoleccion2Atiro2.getHeading(),
                        posicionTiro2ARecoleccion3.getHeading())
                .build();

        recoleccion3 = follower.pathBuilder()
                .addPath(new BezierLine(posicionTiro2ARecoleccion3, posicionFinRecoleccion3))
                .setLinearHeadingInterpolation(posicionTiro2ARecoleccion3.getHeading(),
                        posicionFinRecoleccion3.getHeading())
                .build();

        recoleccion3Atiro3 = follower.pathBuilder()
                .addPath(new BezierLine(posicionFinRecoleccion3, posicionRecoleccion3Atiro3))
                .setLinearHeadingInterpolation(posicionFinRecoleccion3.getHeading(),
                        posicionRecoleccion3Atiro3.getHeading())
                .build();

        fin = follower.pathBuilder()
                .addPath(new BezierCurve(posicionRecoleccion3Atiro3, control3, posicionFin))
                .setLinearHeadingInterpolation(posicionRecoleccion3Atiro3.getHeading(),
                        posicionFin.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (estado) {
            case INICIO_A_RECOLECCION:
                follower.followPath(inicioARecoleccion1, true);
                setPathState(RECOLECCION);
                break;

            case RECOLECCION:
                if (!follower.isBusy()) {
                    follower.followPath(recoleccion1, true);
                    setPathState(RECOLECCION1_A_LANZAMIENTO1);
                }
                break;

            case RECOLECCION1_A_LANZAMIENTO1:
                if (!follower.isBusy()){
                    follower.followPath(recoleccion1Atiro1, true);
                    setPathState(LANZAMIENTO1_A_RECOLECCION2);
                }
                break;

            case LANZAMIENTO1_A_RECOLECCION2:
                if (!follower.isBusy()){
                    follower.followPath(tiro1ARecoleccion2, true);
                    setPathState(RECOLECCION2);
                }
                break;

            case RECOLECCION2:
                if (!follower.isBusy()){
                    follower.followPath(recoleccion2, true);
                    setPathState(RECOLECCION2_A_LANZAMIENTO2);
                }
                break;

            case RECOLECCION2_A_LANZAMIENTO2:
                if (!follower.isBusy()){
                    follower.followPath(recoleccion2Atiro2, true);
                    setPathState(LANZAMIENTO2_A_RECOLECCION3);
                }
                break;

            case LANZAMIENTO2_A_RECOLECCION3:
                if (!follower.isBusy()){
                    follower.followPath(tiro2ARecoleccion3, true);
                    setPathState(RECOLECCION3);
                }
                break;

            case RECOLECCION3:
                if (!follower.isBusy()){
                    follower.followPath(recoleccion3, true);
                    setPathState(RECOLECCION3_A_LANZAMIENTO3);
                }
                break;

            case RECOLECCION3_A_LANZAMIENTO3:
                if (!follower.isBusy()){
                    follower.followPath(recoleccion3Atiro3, true);
                    setPathState(FIN);
                }
                break;

            case FIN:
                if (!follower.isBusy()){
                    follower.followPath(fin, true);
                    setPathState(DONE);
                }
                break;

            case DONE:
                if (!follower.isBusy()){
                    follower.breakFollowing();
                }
                telemetry.addLine("DONE");
                telemetry.update();
                break;
        }
    }

    public void setPathState(int nuevoEstado) {
        estado = nuevoEstado;
        if (pathTimer != null) pathTimer.resetTimer();
    }

    // ---------------- PRE-LOGICA ----------------

    private void preUpdate() {
        switch (preEstado) {
            case preLeerTagObelisco:
                leerTag();
                break;

            case preAlinear:
                if (alinear() || pasoTiempo(alignStart, timeutAlinearMs)) {
                    centrarLanzadorServos();
                    preEstado = preDisparar;
                    shootState = disparadorEmpieza;
                    disparosHechos = 0;
                }
                break;

            case preDisparar:
                if (disparar()) {
                    preEstado = preListo;
                }
                break;

            case preListo:
                break;
        }
    }

    private void leerTag() {
        LLResult res = limaluz.getLatestResult();
        if (res != null && res.isValid() && res.getStaleness() <= 10) {
            List<LLResultTypes.FiducialResult> tags = res.getFiducialResults();
            if (tags != null && !tags.isEmpty()) {
                leidoID = tags.get(0).getFiducialId();
            }
        }

        if (pasoTiempo(preStart, p5Ms)) {
            limaluz.pipelineSwitch(6);
            centrarLanzadorServos();
            preEstado = preAlinear;
            alignStart = System.nanoTime();
        }
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
            return false;
        }

        double error = -tx; // referencia = 0

        if (Math.abs(error) < tolerancia) {
            centrarLanzadorServos();
            pidTimer = 0;
            errorAnterior = 0.0;
            return true;
        }

        long now = System.nanoTime();
        double dt = (pidTimer == 0) ? 0.02 : (now - pidTimer) / 1e9;
        pidTimer = now;

        dt = Range.clip(dt, 0.005, 0.05);

        double derivada = (error - errorAnterior) / dt;
        errorAnterior = error;

        double salida = (kP * error) + (kD * derivada);
        salida = Range.clip(salida, -SERVO_RANGE, SERVO_RANGE);

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

    // ---------------- DISPARO ----------------

    private int calcularVelocidadDisparo() {
        pinpoint.update();

        double pos_Y = pinpoint.getPosY(DistanceUnit.CM);
        double pos_X = pinpoint.getPosX(DistanceUnit.CM);

        Theta0 = Math.atan((2*H2/xb) - Math.tan(Math.toRadians(Thetai)));
        direccion = ((68 - Math.toDegrees(Theta0))/38)*0.2;

        Direccion.setPosition(direccion);

        double deltaX = goalX - pos_X;
        double deltaY = goalY - pos_Y;

        xb = (Math.sqrt(deltaX * deltaX + deltaY * deltaY)) / 100.0;

        return (int)(1436.44779 * Math.pow(1.08097, xb));
    }

    private boolean disparar() {
        int v = calcularVelocidadDisparo() + 50;

        switch (shootState) {
            case disparadorEmpieza:
                if (disparosHechos == 0) {
                    if (lista_MOR.get(0)) {
                        Licuadora.setPosition(posX);
                        modo = Modo.X;
                    } else {
                        Licuadora.setPosition(posB);
                        modo = Modo.B;
                    }
                }

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
                        if (lista_MOR.get(1) && modo != Modo.X) {
                            Licuadora.setPosition(posX);
                            modo = Modo.X;
                        } else if (lista_MOR.get(1)) {
                            Licuadora.setPosition(posA);
                            modo = Modo.A;
                        } else {
                            Licuadora.setPosition(posB);
                            modo = Modo.B;
                        }
                    } else if (disparosHechos == 2) {
                        if (lista_MOR.get(2)) {
                            Licuadora.setPosition(posA);
                            modo = Modo.A;
                        } else {
                            Licuadora.setPosition(posB);
                            modo = Modo.B;
                        }
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


    private void determinatMor(){
        if (leidoID == 21) {
            lista_MOR.add(false);
            lista_MOR.add(true);
            lista_MOR.add(true);

        } else if (leidoID == 22) {
            lista_MOR.add(true);
            lista_MOR.add(false);
            lista_MOR.add(true);

        } else if (leidoID == 23) {
            lista_MOR.add(true);
            lista_MOR.add(true);
            lista_MOR.add(false);
        }
    }

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

    private void actualizarDeteccionColor() {
        detectado = "Nada";

        if (portalColor != null && sensorColor != null) {
            PredominantColorProcessor.Result resultColor = sensorColor.getAnalysis();

            if (resultColor != null) {
                if (resultColor.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN) {
                    detectado = "Verde";
                } else if (resultColor.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) {
                    detectado = "Purpura";
                }
            }
        }
    }

    private void resetSlots() {
        slotA = "Nada";
        slotB = "Nada";
        slotX = "Nada";
        contador = 0;
        lastPiecePresent = false;

        Licuadora.setPosition(posX);
        modo = Modo.X;
    }

    private void guardarDetectadoEnSlot() {

        if ("Nada".equals(detectado)) return;
        if (contador >= 3) return;

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

    private void intentarGuardarDetectado() { // vale la pena resaltar el INTENTAR
        boolean piecePresent = !"Nada".equals(detectado);

        if (piecePresent && !lastPiecePresent && contador < 3) {
            guardarDetectadoEnSlot();
        }

        lastPiecePresent = piecePresent;
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
        Licuadora = hardwareMap.get(Servo.class, "Licuadora");

        Lanzador_I = hardwareMap.get(Servo.class, "Lanzador_I");
        Lanzador_D = hardwareMap.get(Servo.class, "Lanzador_D");

        M_lanzador1 = hardwareMap.get(DcMotorEx.class, "M_lanzador1");
        M_lanzador2 = hardwareMap.get(DcMotorEx.class, "M_lanzador2");

        M_lanzador1.setVelocityPIDFCoefficients(105, 0.0, 0.1, 12.0);

        limaluz = hardwareMap.get(Limelight3A.class, "limaluz");
        limaluz.setPollRateHz(90);
        limaluz.pipelineSwitch(5);
        limaluz.start();

        Direccion.setDirection(Servo.Direction.REVERSE);

        Levantador.setPosition(0.6);

        Direccion.setPosition(0);
        centrarLanzadorServos();

        configurePinpoint();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 56, 8, AngleUnit.DEGREES, 90));
        pinpoint.update();

        preEstado = preLeerTagObelisco;

        buildPaths();
        follower.setPose(posicionInicio);

        camaraColor();

        Licuadora.setPosition(posX);
        modo = Modo.X;

        resetSlots();

    }

    @Override
    public void start() {
        setPathState(estado);

        limaluz.pipelineSwitch(5);
        leidoID = -1;

        preEstado = preLeerTagObelisco;
        preStart = System.nanoTime();

        alignStart = 0;

        shootState = disparadorEmpieza;
        disparosHechos = 0;
    }

    @Override
    public void loop() {
        determinatMor();
        calcularVelocidadDisparo();
        preUpdate();

        if (preEstado == preListo) {
            follower.update();
            statePathUpdate();
        }

//        actualizarDeteccionColor();
//        intentarGuardarDetectado();

        telemetry.addData("PreEstado", preEstado);
        telemetry.addData("Tag ID (P5)", leidoID);
        telemetry.addData("Servo Corr", correccion);
        telemetry.addData("Path Estado", estado);
        telemetry.update();
    }

    public void configurePinpoint(){
        pinpoint.setOffsets(-110/2.54, -160/2.54, DistanceUnit.MM);

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
    }
}
