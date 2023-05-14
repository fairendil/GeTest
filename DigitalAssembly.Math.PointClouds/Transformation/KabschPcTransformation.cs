using DigitalAssembly.Math.Common;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Statistics;
using static System.Math;

namespace DigitalAssembly.Math.PointClouds.Transformation;

/// <summary>
/// Kabsch algorithm for estimaing transformation between two point clouds
/// </summary>
public class KabschPcTransformation : IPcTransformation
{
    public KabschPcTransformation() { }

    // TODO: research other fitting algorithms for point clouds
    // TODO: what if model not found?
    /// <summary>
    /// Алгоритм Кабша: вычисление матрицы перехода от одного облака точек (системы координат) к другому (другой системе координат)
    /// </summary>
    /// <typeparam name="PT">Point of type Point3D</typeparam>
    /// <param name="initialPoints"></param>
    /// <param name="targetPoints"></param>
    /// <returns></returns>
    /// <exception cref="ArgumentException"></exception>
    public TransformationFittingResult Estimate<PT>(List<PT> initialPoints, List<PT> targetPoints)
        where PT : Point3D<PT>
    {
        if (initialPoints.Count != targetPoints.Count)
        {
            throw new ArgumentException("Number of initialPoints and targetPoints should be same");
        }

        int pointsCount = initialPoints.Count;
        Matrix<double> inMatrix = Matrix<double>.Build.Dense(pointsCount, 3);
        Matrix<double> outMatrix = Matrix<double>.Build.Dense(pointsCount, 3);
        for (int i = 0; i < pointsCount; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                inMatrix[i, j] = initialPoints[i].Coordinate[j];
                outMatrix[i, j] = targetPoints[i].Coordinate[j];
            }
        }

        Vector<double> inCentroid = inMatrix.ColumnSums() / pointsCount;
        Vector<double> tgtCentroid = outMatrix.ColumnSums() / pointsCount;
        for (int col = 0; col < pointsCount; col++)
        {
            for (int j = 0; j < 3; j++)
            {
                inMatrix[col, j] -= inCentroid[j];
                outMatrix[col, j] -= tgtCentroid[j];
            }
        }

        Matrix<double> cov = inMatrix.TransposeThisAndMultiply(outMatrix);
        MathNet.Numerics.LinearAlgebra.Factorization.Svd<double> svdDecompose = cov.Svd();
        double det = Sign((svdDecompose.U * svdDecompose.VT).Transpose().Determinant());
        Matrix<double> I = Matrix<double>.Build.DenseIdentity(3, 3);
        I[2, 2] = det;
        Matrix<double> R = (svdDecompose.U * I * svdDecompose.VT).Transpose();
        Vector<double> translation = tgtCentroid - (R * inCentroid);
        Matrix<double> resultTransformation = Matrix<double>.Build.Dense(4, 4);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                resultTransformation[i, j] = R[i, j];
            }

            resultTransformation[i, 3] = translation[i];
        }

        resultTransformation[3, 3] = 1;
        List<Vector<double>> visibleTransformedPoints = new();
        foreach (PT point in initialPoints)
        {
            visibleTransformedPoints.Add((resultTransformation * point.Homogenous).SubVector(0,3));
        }

        List<double> checkBackPoints = new();
        for (int i = 0; i < visibleTransformedPoints.Count; i++)
        {
            checkBackPoints.Add((visibleTransformedPoints[i] - targetPoints[i].Coordinate).L2Norm());
        }

        return new(resultTransformation, checkBackPoints.Mean());
    }
}
