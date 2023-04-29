package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Var.CV_detectionType;
import static org.firstinspires.ftc.teamcode.Var.CV_kernel_pult_size;
import static org.firstinspires.ftc.teamcode.Var.CV_rect_x1;
import static org.firstinspires.ftc.teamcode.Var.CV_rect_x2;
import static org.firstinspires.ftc.teamcode.Var.CV_rect_y1;
import static org.firstinspires.ftc.teamcode.Var.CV_rect_y2;
import static org.firstinspires.ftc.teamcode.VarPipelineStalp.GaCV_detectionType;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import static org.firstinspires.ftc.teamcode.VarPipelineStalp.*;

public class PipelineStalp extends OpenCvPipeline {
    //stabileste forma detectorului
    private final int elementType = Imgproc.CV_SHAPE_RECT;
    //asta e un dreptunghi(Rect = dreptunghi pentru webcam)
    private Rect dreptunghi;
    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */

    @Override
    //mat = foaie de desen pentru webcam
    public Mat processFrame(Mat input) {
        try {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */
            //face un patrat de latura kernel_pult_size si cu ancora in centru
            Mat element = Imgproc.getStructuringElement(elementType, new Size(2 * CV_kernel_pult_size + 1, 2 * CV_kernel_pult_size + 1),
                    new Point(CV_kernel_pult_size, CV_kernel_pult_size));
            //creeaza o copie a imaginii de pe webcam
            Mat original = input.clone();

            //Scalari de HSV(totusi H e jumate din valorile de pe color picker)
            Scalar scalarLowerHSV, scalarUpperHSV;

            //daca e day da valorile de day, daca e night da valorile de night
            if (GaCV_detectionType == VarPipelineStalp.DetectionTypes.DAY) {
                scalarLowerHSV = new Scalar(GaDay_Hlow, GaDay_Slow, GaDay_Vlow);
                scalarUpperHSV = new Scalar(GaDay_Hhigh, GaDay_Shigh, GaDay_Vhigh);
            }
            else {
                scalarLowerHSV = new Scalar(GaNight_Hlow, GaNight_Slow, GaNight_Vlow);
                scalarUpperHSV = new Scalar(GaNight_Hhigh, GaNight_Shigh, GaNight_Vhigh);
            }

            //asta converteste culorile din input de la RGB la HSV
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);

            //asta face ca culorile din input sa fie intre scalarLowerHSV si scalarUpperHSV
            Core.inRange(input, scalarLowerHSV, scalarUpperHSV, input);

            //aceasta parte reduce partile neregulate din imagine
            //erode micsoreaza pixelii, dilate mareste pixelii
            Imgproc.erode(input, input, element);
            Imgproc.dilate(input, input, element);
            Imgproc.dilate(input, input, element);
            Imgproc.erode(input, input, element);

            //aceasta linie de cod face un dreptunghi cat webcam-ul de mare si negru
            Mat rect = new Mat(input.rows(), input.cols(), input.type(), Scalar.all(0));

            //face un dreptunghi care acopera toata aria de detectare cu alb
            Imgproc.rectangle(
                    rect,
                    new Point(CV_rect_x1, CV_rect_y1),
                    new Point(CV_rect_x2, CV_rect_y2),
                    new Scalar(255),
                    Imgproc.FILLED
            );
            //aici se converteste culoarea in alb-negru, albul fiind chestiile detectate
            Core.bitwise_and(input, rect, input);

            //asta declara o lista de contururi;
            List<MatOfPoint> contours = new ArrayList<>();

            //input e imaginea, contours este lista de contururi, retr_list doar da toate contururile, chain_approx_simple face ca formele sa fie facute numai din varfurile lor
            Imgproc.findContours(input, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            //Aici se sorteaza contururile in mod descrescator;
            Collections.sort(contours, new Comparator<MatOfPoint>() {
                @Override
                public int compare(MatOfPoint matOfPoint, MatOfPoint t1) {
                    return (int) (Imgproc.contourArea(t1) - Imgproc.contourArea(matOfPoint));
                }
            });
            //aici se face un dreptunghi din conturul cel mai mare
            if (!contours.isEmpty()) {
                setRect(Imgproc.boundingRect(contours.get(0)));
            }
            //aici se converteste imaginea de la alb-negru la  color inapoi
            Imgproc.cvtColor(input, input, Imgproc.COLOR_GRAY2RGBA);

            //reface imaginea originala cu tot cu partea de culoarea potrivita diferita
            Core.bitwise_or(input, original, input);

            //deseneaza toate contururile, con
            // cotourldx=-1 inseamna ca sunt desenate TOATE contururile.
            Imgproc.drawContours(input, contours, -1, new Scalar(0, 255, 0), 4);

            //aici se deseneaza dreeptunghiul care arata aria de detectare
            Imgproc.rectangle(
                    input,
                    new Point(CV_rect_x1, CV_rect_y1),
                    new Point(CV_rect_x2, CV_rect_y2),
                    new Scalar(255, 127, 0), 4);

            //asta umple forma pe care a detectat-o
            Imgproc.rectangle(
                    input,
                    getRect(),
                    new Scalar(0, 255, 255), 4);
            //aici se elimina toate modificarile din aceste mat-uri;
            original.release();
            rect.release();
            //se returneaza input-ul modificat
        }
        catch (Exception E){

        }
        return input;
    }

    public void setRect(Rect rect) {
        this.dreptunghi = rect;
    }

    public Rect getRect() {
        return dreptunghi;
    }
}