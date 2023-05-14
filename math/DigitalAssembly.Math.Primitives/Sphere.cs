using System;
using static System.Math;
using DigitalAssembly.Math.Matrices;
using MathNet.Numerics.LinearAlgebra;
using System.Threading.Tasks;
using Aladdin.HASP.Envelope;

namespace DigitalAssembly.Math.Primitives;

public class Sphere
{
    public double X { get; set; }
    public double Y { get; set; }
    public double Z { get; set; }
    public double Radius { get; set; }

    //For Entering Points
    private Vector<double> _sphereVec;
    private Point[] _points;
    private Vector<double> _valRMS;
    private Matrix<double> _dValRMS;

    public Sphere()
    {
        X = 0;
        Y = 0;
        Z = 0;
        Radius = 0;
    }

    public Sphere(double xIn, double yIn, double zIn, double rIn)
    {
        X = xIn;
        Y = yIn;
        Z = zIn;
        Radius = rIn;
    }

    private void UpdateFuncAndDiff(int numPoints) => Parallel.For(0, numPoints, IterationCalcPoint);

    void IterationCalcPoint(int i)
    {
        double divSphereX = _sphereVec[0] - _points[i].X;
        double divSphereY = _sphereVec[1] - _points[i].Y;
        double divSphereZ = _sphereVec[2] - _points[i].Z;

        _valRMS[i] =
            Pow(divSphereX, 2) + Pow(divSphereY, 2) +
            Pow(divSphereZ, 2) - Pow(_sphereVec[3], 2);
        _dValRMS[i, 0] = 2 * divSphereX;
        _dValRMS[i, 1] = 2 * divSphereY;
        _dValRMS[i, 2] = 2 * divSphereZ;
        _dValRMS[i, 3] = -2 * _sphereVec[3];
    }

    public static Sphere EnterPoints(Point[] points)
    {
        Sphere sphereOut = new();

        int numPoints = points.Length;

        VectorBuilder<double> V = Vector<double>.Build;
        Vector<double> SphereVecStart = V.Dense(4,0);
        SphereVecStart[3] = 1;

        sphereOut.InitOptimizationValue(SphereVecStart, points);

        Vector<double> SphereVecOld = SphereVecStart;

        Vector<double> valRMS;
        Matrix<double> dValRMS;

        sphereOut.UpdateFuncAndDiff(numPoints);
        (valRMS, dValRMS) = sphereOut.GetOptimizationValue();
        Vector<double> bVec = (dValRMS * SphereVecOld) - valRMS;
        Vector<double> SphereVecNew = dValRMS.Solve(bVec);

        int i = 2;
        double eps = Pow(10, -6);

        while ((SphereVecNew - SphereVecOld).Norm(2) > eps && i < 100)
        {
            SphereVecOld = SphereVecNew;

            sphereOut.SetOptimizationValue(SphereVecOld);
            sphereOut.UpdateFuncAndDiff(numPoints);
            (valRMS, dValRMS) = sphereOut.GetOptimizationValue();
            bVec = (dValRMS * SphereVecOld) - valRMS;
            SphereVecNew = dValRMS.Solve(bVec);
            i++;
        }

        sphereOut.X = SphereVecNew[0];
        sphereOut.Y = SphereVecNew[1];
        sphereOut.Z = SphereVecNew[2];
        sphereOut.Radius = Abs(SphereVecNew[3]);

        return sphereOut;
    }

    public void InitOptimizationValue(Vector<double> sphereVec,
        Point[] points)
    {
        _sphereVec = sphereVec;
        _points = points;

        int numPoints = points.Length;

        MatrixBuilder<double> M = Matrix<double>.Build;
        VectorBuilder<double> V = Vector<double>.Build;
        _valRMS = V.Dense(numPoints, 0);
        _dValRMS = M.Dense(numPoints, 4, 0);
    }

    public void SetOptimizationValue(Vector<double> sphereVec) => _sphereVec = sphereVec;

    public (Vector<double>, Matrix<double>) GetOptimizationValue() => (_valRMS, _dValRMS);

}
