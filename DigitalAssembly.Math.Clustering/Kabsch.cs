

using DigitalAssembly.Math.Common;
using MathNet.Numerics.LinearAlgebra;

namespace DigitalAssembly.Math.Clustering;

// Алгоритм Кабша: вычисление матрицы перехода от одного облака точек (системы координат) к другому (другой системе координат)
public class Kabsch<T>
    where T: Point3D<T>
{
    private readonly double Tolerance;
    private readonly double Similarity;

    public Kabsch(double tolerance, double similarity) 
    {
        Tolerance = tolerance;
        Similarity = similarity;
    }

    public Matrix<double> EstimateTransformation<PT>(List<PT> initialPoints, List<PT> targetPoints)
        where PT : Point3D<T>
    {

    }
}
