using DigitalAssembly.Math.Common;
using DigitalAssembly.Math.PointClouds.Classification;
using DigitalAssembly.Math.PointClouds.Transformation;
using MathNet.Numerics.LinearAlgebra;
using System.Diagnostics;

namespace DigitalAssembly.Math.PointClouds;

public class ClassifyAndComputeTransformation
{
    private readonly IPcClassification _classificator;
    private readonly IPcTransformation _transformationFinder;
    private readonly double _classificationMeanSquaredFittingErrorTolerance;

    public ClassifyAndComputeTransformation(double tolerance, double similarity)
    {
        _classificator = new ChineseСrosswordClassification(tolerance, similarity);
        _transformationFinder = new KabschPcTransformation();
        // TODO: Add tolerance from globals
        _classificationMeanSquaredFittingErrorTolerance = tolerance;
    }

    /// <summary>
    /// Вычисление матрицы перехода от одного облака точек (системы координат) к другому (другой системе координат)
    /// </summary>
    /// <typeparam name="PT">Point of type Point3D</typeparam>
    /// <param name="initialPoints"></param>
    /// <param name="targetPoints"></param>
    /// <returns></returns>
    public PointCloudClassificationResult<PT> Estimate<PT>(List<PT> initialPoints, List<PT> targetPoints)
        where PT : Point3D<PT>
    {
        (List<PT> initialPointsFound, List<PT> targetPointsFound) = _classificator.SelectPoints(initialPoints, targetPoints);
        if (_transformationFinder is KabschPcTransformation)
        {
            if (initialPointsFound.Count == 0 || targetPointsFound.Count == 0)
            {
                return new(Matrix<double>.Build.DenseDiagonal(4, 4, 1), new(), new(), double.NaN, ClassificationStatus.NotClassified);
            }

            TransformationFittingResult resultTransformation = _transformationFinder.Estimate(initialPointsFound, targetPointsFound);
            ClassificationStatus status = resultTransformation.MeanSquaredFittingError < _classificationMeanSquaredFittingErrorTolerance
                                            ? ClassificationStatus.Good
                                            : ClassificationStatus.Bad;
            return new(resultTransformation.Transformation, targetPointsFound, initialPointsFound, resultTransformation.MeanSquaredFittingError, status);
        }

        throw new UnreachableException();
    }

    public (List<PT> initialPointsFound, List<PT> targetPointsFound) SelectPoints<PT>(List<PT> initialPoints, List<PT> targetPoints)
        where PT : Point3D<PT> => _classificator.SelectPoints(initialPoints, targetPoints);
}
