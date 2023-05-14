using MathNet.Numerics.Distributions;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Optimization;
using MathNet.Spatial.Units;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static System.Math;

namespace DigitalAssembly.Math.Primitives;

public class Cylinder
{

    public double Radius { get; set; }

    public double X { get; set; }
    public double Y { get; set; }
    public double Z { get; set; }
    public double I { get; set; }
    public double J { get; set; }
    public double K { get; set; }

    public Cylinder()
    {
        Radius = 0;
        X = 0;
        Y = 0;
        Z = 0;
        I = 0;
        J = 0;
        K = 0;
    }

    public Cylinder(double radiusIn, double xIn, double yIn, double zIn, double iIn, double jIn, double kIn)
    {
        Radius = radiusIn;
        X = xIn;
        Y = yIn;
        Z = zIn;
        I = iIn;
        J = jIn;
        K = kIn;
    }

    /*Return the direction vector of a cylinder defined
        by the spherical coordinates theta and phi.*/
    private static Vector<double> Direction(double theta, double phi)
    {
        Vector<double> result = Vector<double>.Build.Dense(3);
        result[0] = Cos(phi) * Sin(theta);
        result[1] = Sin(phi) * Sin(theta);
        result[2] = Cos(theta);
        return result;
    }

    //Return the projection matrix  of a direction w.
    private static Matrix<double> ProjectionMatrix(Vector<double> w)
    {
        Matrix<double> MatrixRowVector = Matrix<double>.Build.Dense(3, 3);
        Matrix<double> MatrixColumnVector = Matrix<double>.Build.Dense(3, 3);

        MatrixRowVector.SetRow(0, w);
        MatrixColumnVector.SetColumn(0, w);

        return Matrix<double>.Build.DenseIdentity(3, 3) - (MatrixColumnVector * MatrixRowVector);
    }

    //Return the skew matrix of a direction w.
    private static Matrix<double> SkewMatrix(Vector<double> w)
    {
        return Matrix<double>.Build.DenseOfArray(new[,] {{ 0,  w[2], -w[1]},
                            {-w[2], 0, w[0]},
                            {w[1],  -w[0], 0.0}});
    }

    //Return the matrix SumY from a list of Y vectors.
    private static Matrix<double> CalcSumY(Vector<double>[] Ys)
    {
        Matrix<double> SumY = Matrix<double>.Build.Dense(3, 3, 0);
        Matrix<double> MatrixRowVector = Matrix<double>.Build.Dense(3, 3);
        Matrix<double> MatrixColumnVector = Matrix<double>.Build.Dense(3, 3);

        for (int i = 0; i < Ys.Length; i++)
        {
            MatrixRowVector.SetRow(0, Ys[i]);
            MatrixColumnVector.SetColumn(0, Ys[i]);

            SumY += MatrixColumnVector * MatrixRowVector;
        }

        return SumY;
    }

    //Return the A_hat matrix of A given the skew matrix S.
    private static Matrix<double> CalcAhat(Matrix<double> A, Matrix<double> S) => S * (A * S.Transpose());

    /*Translate the center of mass(COM) of the data to the origin.
    Return the prossed data and the shift of the COM*/
    private static Vector<double>[] PreprocessData(Vector<double>[] Xs_raw)
    {
        Vector<double> Xs_raw_mean = Vector<double>.Build.Dense(3, 0);

        for (int i = 0; i < Xs_raw.Length; i++)
        {
            Xs_raw_mean += Xs_raw[i];
        }

        Xs_raw_mean /= Xs_raw.Length;

        for (int i = 0; i < Xs_raw.Length; i++)
        {
            Xs_raw[i] -= Xs_raw_mean;
        }

        return Xs_raw;
    }

    /*Calculate the center of mass(COM)*/
    private static Vector<double> CalculateCOM(Vector<double>[] Xs_raw)
    {
        Vector<double> Xs_raw_mean = Vector<double>.Build.Dense(3, 0);

        for (int i = 0; i < Xs_raw.Length; i++)
        {
            Xs_raw_mean += Xs_raw[i];
        }

        Xs_raw_mean /= Xs_raw.Length;

        return Xs_raw_mean;
    }

    private static Vector<double> CalculateC(Vector<double> w, Vector<double>[] Xs)
    {

        Matrix<double> P = ProjectionMatrix(w);

        Vector<double>[] Ys = new Vector<double>[Xs.Length];

        for (int i = 0; i < Xs.Length; i++)
        {
            Ys[i] = Xs[i] * P;
        }

        Matrix<double> A = CalcSumY(Ys);
        Matrix<double> A_hat = CalcAhat(A, SkewMatrix(w));

        Matrix<double> At = A * A_hat;
        double sumDiagonal = 0;

        sumDiagonal += At[0, 0];
        sumDiagonal += At[1, 1];
        sumDiagonal += At[2, 2];

        Vector<double> v = Vector<double>.Build.Dense(3);

        for (int i = 0; i < Xs.Length; i++)
        {
            v += Ys[i] * Ys[i] * Ys[i];
        }

        v = v * A_hat / sumDiagonal;

        return v;
    }

    private static double CalculateR(Vector<double> w, Vector<double>[] Xs)
    {

        Matrix<double> P = ProjectionMatrix(w);
        Vector<double> c = CalculateC(w, Xs);

        double result = 0;

        for (int i = 0; i < Xs.Length; i++)
        {
            result += (c - Xs[i]) * (P * (c - Xs[i]));
        }

        result = Sqrt(result / Xs.Length);
        return result;
    }

    /*Calculate the G function given a cylinder direction w and a
    list of data points Xs to be fitted.*/
    private static double G(Vector<double> w, Vector<double>[] Xs)
    {
        Matrix<double> P = ProjectionMatrix(w);

        Vector<double>[] Ys = new Vector<double>[Xs.Length];

        for (int i = 0; i < Xs.Length; i++)
        {
            Ys[i] = Xs[i] * P;
        }

        Matrix<double> A = CalcSumY(Ys);
        Matrix<double> A_hat = CalcAhat(A, SkewMatrix(w));

        double u = 0;
        for (int i = 0; i < Xs.Length; i++)
        {
            u += Ys[i] * Ys[i];
        }

        u /= Xs.Length;

        Vector<double> v = Vector<double>.Build.Dense(3);
        Matrix<double> At = A * A_hat;
        double sumDiagonalMatrix = 0;

        sumDiagonalMatrix += At[0, 0];
        sumDiagonalMatrix += At[1, 1];
        sumDiagonalMatrix += At[2, 2];
        for (int i = 0; i < Xs.Length; i++)
        {
            v += Ys[i] * Ys[i] * Ys[i];
        }

        v = v * A_hat / sumDiagonalMatrix;

        double result = 0;
        for (int i = 0; i < Xs.Length; i++)
        {
            result += Pow((Ys[i] * Ys[i]) - u - (2 * Ys[i] * v), 2);
        }

        return result;
    }

    public static Cylinder Fit(Vector<double>[] Points)
    {
        Cylinder cylinderOut = new();

        Vector<double> w;
        Vector<double>[] PreprocessPoints = PreprocessData(Points);

        IObjectiveFunction functionForOptimization = ObjectiveFunction.Value(x => G(Direction(x[0], x[1]), PreprocessPoints));

        Vector<double> initalGuess = Vector<double>.Build.Dense(2);
        initalGuess[0] = 1.5707963267948966;
        initalGuess[1] = 1.5707963267948966;

        MinimizationResult resultFit = NelderMeadSimplex.Minimum(functionForOptimization, initalGuess, 1e-12, 1000000);

        w = Direction(resultFit.MinimizingPoint[0], resultFit.MinimizingPoint[1]);

        cylinderOut.Radius = CalculateR(w, PreprocessPoints);

        cylinderOut.I = w[0];
        cylinderOut.J = w[1];
        cylinderOut.K = w[2];

        cylinderOut.X = (CalculateCOM(Points) + CalculateC(w, PreprocessPoints))[0];
        cylinderOut.Y = (CalculateCOM(Points) + CalculateC(w, PreprocessPoints))[1];
        cylinderOut.Z = (CalculateCOM(Points) + CalculateC(w, PreprocessPoints))[2];

        return cylinderOut;
    }
}