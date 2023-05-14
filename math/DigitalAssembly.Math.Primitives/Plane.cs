using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra.Factorization;

namespace DigitalAssembly.Math.Primitives;

public class Plane
{
    public double A { get; set; }
    public double B { get; set; }
    public double C { get; set; }
    public double D { get; set; }

    public Plane()
    {
        A = 0.0;
        B = 0.0;
        C = 0.0;
        D = 0.0;

    }

    public Plane(double aIn, double bIn, double cIn, double dIn)
    {
        A = aIn;
        B = bIn;
        C = cIn;
        D = dIn;

    }

    public static Plane Fit(Point[] points)
    {
        Plane planeOut = new();

        double[,] matrixCoordinate = new double[3, points.Length];
        for (int k = 0; k < 3; k++)
        {
            for (int j = 0; j < points.Length; j++)
            {

                if (k == 0)
                {
                    matrixCoordinate[k, j] = points[j].X;
                }

                if (k == 1)
                {
                    matrixCoordinate[k, j] = points[j].Y;
                }

                if (k == 2)
                {
                    matrixCoordinate[k, j] = points[j].Z;
                }
            }    
        }

        double sum = 0;
        double[] meanVector = new double[3];

        for (int k = 0; k < 3; k++)
        {
            for (int j = 0; j < matrixCoordinate.GetLength(1); j++)
            {
                sum += matrixCoordinate[k, j];
            }

            meanVector[k] = sum / matrixCoordinate.GetLength(1);
            sum = 0;

            for (int j = 0; j < matrixCoordinate.GetLength(1); j++)
            {
                matrixCoordinate[k, j] -= meanVector[k];
            }
        }

        Svd<double> svd = DenseMatrix.OfArray(matrixCoordinate).Svd(true);

        planeOut.A = -svd.U[0, 2];
        planeOut.B = -svd.U[1, 2];
        planeOut.C = -svd.U[2, 2];
        planeOut.D = 0;

        for (int k = 0; k < 3; k++)
        {
            planeOut.D += svd.U[k, 2] * meanVector[k];
        }

        return planeOut;
    }
}
