using DigitalAssembly.GoldenEye.Classifier;
using DigitalAssembly.GoldenEye.DB;
using DigitalAssembly.GoldenEye.Geometry;
using DigitalAssembly.GoldenEye.Objects;
using DigitalAssembly.GoldenEye.Serializers;
using DigitalAssembly.Math.Common;
using DigitalAssembly.Math.PointClouds;
using DigitalAssembly.Photogrammetry;
using DigitalAssembly.Photogrammetry.Geometry.CoordinateSystems;
using DigitalAssembly.Photogrammetry.Serializers;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Statistics;
using static System.Math;

namespace DigitalAssembly.GoldenEye.UnitTests.PointCloudClassification;

/// <summary>
/// Tolerance for TransformationOfPointCloud should be N times higher than measurement system tolerance. 
/// While TransformationOfPointCloud tolerance increasing, number of found points would be higher, but tolerance of found transformation might decrease
/// </summary>
public class ClassifyAdapterAndTable
{
    private ClassifyAndComputeTransformation _transformationFinder;
    private const int DOUBLE_COUNT = 4;
    // With 'tolerance = 1' L2Norm for visible points < 0.25
    private const double _tolerance = 1;
    private const double _similarity = 0.85;
    private const double _precitionPoints = 0.5;

    [SetUp]
    public void Setup() => _transformationFinder = new(_tolerance, _similarity);

    private static MarkPoint<T>[] LoadPointsFromFile<T>(string filename)
        where T: Point3D<T>
    {
        List<MarkPoint<T>> points = new();
        double[][] doubleValues = DoubleCsvSerializer.LoadFromFile(filename, DOUBLE_COUNT, @"\,?\s+");
        for (int i = 0; i < doubleValues.Length; ++i)
        {
            double[] doubles = doubleValues[i];
            if (Abs(doubles[0] % 1) > double.Epsilon)
            {
                throw new SerializerException($"Mark code in line '{i}' is not int value");
            }

            int code = (int)doubles[0];
            T point = (T)Activator.CreateInstance(typeof(T), doubles[1], doubles[2], doubles[3])!;
            points.Add(MarkPoint<T>.FromCode(code, MarkCodeType.BitCode14b, point));
        }

        return points.ToArray();
    }

    private static List<T> ConvertToPoints<T>(MarkPoint<T>[] points)
        where T: Point3D<T> => points.Select(point => point.Point).ToList();

    private static double TransformPointComputeError<T>(T point, Matrix<double> transformation, T initialPoint)
        where T: Point3D<T>
    {
        T resPoint = ApplyTransformation(point, transformation);
        return (resPoint - initialPoint).L2Norm();
    }

    private static T ApplyTransformation<T>(T point, Matrix<double> transformation)
        where T : Point3D<T>
    {
        Vector<double> resPoint = transformation * point.Homogenous;
        resPoint /= resPoint[3];
        return (T)Activator.CreateInstance(typeof(T), resPoint[0], resPoint[1], resPoint[2])!;
    }

    [Test]
    [TestCase("Points/Reference_SC_self.txt", "Points/Ref+adapt_SC_MI.txt", "Points/Ref_SC_MI.txt")]
    [TestCase("Points/Adapter_SC_self.txt", "Points/Ref+adapt_SC_MI.txt", "Points/Adapter_SC_MI.txt")]
    public void EstimateModelTransformationCheckPoints(string nomimalModelFileName, string measuredDataFileName, string measuredModelFileName)
    {
        List<ModelCsPoint> nominalPoints = ConvertToPoints(LoadPointsFromFile<ModelCsPoint>(nomimalModelFileName));
        List<ModelCsPoint> measuredPoints = ConvertToPoints(LoadPointsFromFile<ModelCsPoint>(measuredDataFileName));
        List<ModelCsPoint> measuredModel = ConvertToPoints(LoadPointsFromFile<ModelCsPoint>(measuredModelFileName));
        PointCloudClassificationResult<ModelCsPoint> classificationResult = _transformationFinder.Estimate<ModelCsPoint>(nominalPoints, measuredPoints);
        Matrix<double> computedTransformation = classificationResult.PointCloudTransformation;
        List<ModelCsPoint> visibleNominalPoints = classificationResult.VisibleInitialPoints;
        (visibleNominalPoints, List<ModelCsPoint> visibleMeasuredModelPoints) = _transformationFinder.SelectPoints(nominalPoints, measuredModel);
        Console.WriteLine($"Number used points: {visibleNominalPoints.Count} from initial {nominalPoints.Count}");
        Console.WriteLine($"Computed transformation:{computedTransformation}");
        List<double> checkBackPoints = new();
        for (int i = 0; i < visibleNominalPoints.Count; i++)
        {
            checkBackPoints.Add(TransformPointComputeError(visibleNominalPoints[i], computedTransformation, visibleMeasuredModelPoints[i]));
            Console.WriteLine($"Back projection error: {checkBackPoints[i]}");
        }

        Console.WriteLine($"Back projection\nmax error {checkBackPoints.Max()}\nmean error {checkBackPoints.Mean()}");
        if (checkBackPoints.Any(i => i > _precitionPoints))
        {
            int numberNotValidPoints = checkBackPoints.Count(i => i > _precitionPoints);
            Console.WriteLine($"Not valid points {numberNotValidPoints} from {visibleNominalPoints.Count}");
            if (numberNotValidPoints / visibleNominalPoints.Count > 0.1)
            {
                Assert.Fail();
            }
        }
        Assert.Pass();
    }

    [Test]
    [TestCase("Points/Reference_SC_self.txt", "Points/Ref+adapt_SC_MI.txt", "Points/Ref_SC_MI.txt")]
    [TestCase("Points/Adapter_SC_self.txt", "Points/Ref+adapt_SC_MI.txt", "Points/Adapter_SC_MI.txt")]
    public void LoadModelEstimateTransformationCheckPoints(string nomimalModelFileName, string measuredDataFileName, string measuredModelFileName)
    {
        MarkPoint<ModelCsPoint>[] modelMarks = LoadPointsFromFile<ModelCsPoint>(nomimalModelFileName);
        MarkPoint<ModelCsPoint>[] measuresMarks = LoadPointsFromFile<ModelCsPoint>(measuredDataFileName);
        MarkPoint<ModelCsPoint>[] measuredModelMarks = LoadPointsFromFile<ModelCsPoint>(measuredModelFileName);
        List<ModelCsPoint> measuredModel = ConvertToPoints(measuredModelMarks);
        AdapterModel model = (AdapterModel)ModelSerializer.LoadAdapterFromFile(nomimalModelFileName);
        List<Model> objects = new()
        {
            model
        };
        ModelsClassifier classifier = new(objects);
        List<ClassificationResult> result = classifier.Classify(measuresMarks);
        Assert.That(result, Has.Count.EqualTo(1), "One model should be found");
        MovableObject foundModel = result.Select(element => element.MeasuredObject).ToList()[0];
        Console.WriteLine($"Mean fitting error: {result.Select(element => element.MeanSquaredFittingError).ToList()[0]}");
        List<MarkPoint<ModelCsPoint>> transformedModelMarks = foundModel.GetTransformedModelPoints();
        List<ModelCsPoint> transformedModelPoints = transformedModelMarks.Select(point => point.Point).ToList();
        (List<ModelCsPoint> visibleTransformedPoints, List<ModelCsPoint> visibleMeasuredModelPoints) 
            = _transformationFinder.SelectPoints<ModelCsPoint>(transformedModelPoints, measuredModel);
        List<double> checkBackPoints = new();
        for (int i = 0; i < visibleTransformedPoints.Count; i++)
        {
            checkBackPoints.Add((visibleTransformedPoints[i] - visibleMeasuredModelPoints[i]).L2Norm());
            Console.WriteLine($"Back projection error: {checkBackPoints[i]}");
        }

        Console.WriteLine($"Back projection\nmax error {checkBackPoints.Max()}\nmean error {checkBackPoints.Mean()}");
        if (checkBackPoints.Any(i => i > _precitionPoints))
        {
            int numberNotValidPoints = checkBackPoints.Count(i => i > _precitionPoints);
            Console.WriteLine($"Not valid points {numberNotValidPoints} from {visibleTransformedPoints.Count}");
            if (numberNotValidPoints / visibleTransformedPoints.Count > 0.1)
            {
                Assert.Fail();
            }
        }
        Assert.Pass();
    }

    [Test]
    [TestCase("Points/Reference_SC_self.txt", "Points/Adapter_SC_self.txt", "Points/Ref+adapt_SC_MI.txt", "Points/Adapter_SC_Ref.txt")]
    public void TestAdapterAndReferenceBackTransformation(string referenceFileName, string adapterFileName, string measuredDataFileName, string adapterInRefCsFileName)
    {
        List<ModelCsPoint> referencePoints = ConvertToPoints(LoadPointsFromFile<ModelCsPoint>(referenceFileName));
        MarkPoint<ModelCsPoint>[] adapterMarks = LoadPointsFromFile<ModelCsPoint>(adapterFileName);
        List<ModelCsPoint> adapterPoints = ConvertToPoints(adapterMarks);
        List<ModelCsPoint> measuredData = ConvertToPoints(LoadPointsFromFile<ModelCsPoint>(measuredDataFileName));
        MarkPoint<ModelCsPoint>[] adapterMarksInRefCs = LoadPointsFromFile<ModelCsPoint>(adapterInRefCsFileName);
        List<ModelCsPoint> adapterPointsInRefCs = ConvertToPoints(adapterMarksInRefCs);

        Console.WriteLine("Adapter classification");
        PointCloudClassificationResult<ModelCsPoint> classificationResult = _transformationFinder.Estimate(adapterPoints, measuredData);
        Matrix<double> adapterTransformation = classificationResult.PointCloudTransformation;
        List<ModelCsPoint> visibleAdapterPoints = classificationResult.VisibleInitialPoints;
        List<ModelCsPoint> visibleInitialAdapterPoints = classificationResult.UsedTargetPoints;
        Console.WriteLine($"Number used points: {visibleInitialAdapterPoints.Count} from initial {adapterPoints.Count}");
        Console.WriteLine($"Computed transformation:{adapterTransformation}");
        List<double> checkBackPoints = new();
        for (int i = 0; i < visibleInitialAdapterPoints.Count; i++)
        {
            checkBackPoints.Add(TransformPointComputeError(visibleInitialAdapterPoints[i], adapterTransformation, visibleAdapterPoints[i]));
        }

        Console.WriteLine($"Back projection\nmax error {checkBackPoints.Max()}\nmean error {checkBackPoints.Mean()}");
        if (checkBackPoints.Any(i => i > _precitionPoints))
        {
            Console.WriteLine($"Not valid points {checkBackPoints.Count(i => i > _precitionPoints)} from {visibleInitialAdapterPoints.Count}");
        }

        Console.WriteLine("\nReference classification");
        classificationResult = _transformationFinder.Estimate<ModelCsPoint>(referencePoints, measuredData);
        Matrix<double> referenceTransformation = classificationResult.PointCloudTransformation;
        List<ModelCsPoint> visibleReferencePoints = classificationResult.UsedTargetPoints;
        List<ModelCsPoint> visibleInitialReferencePoints = classificationResult.VisibleInitialPoints;
        Console.WriteLine($"Number used points: {visibleInitialReferencePoints.Count} from initial {referencePoints.Count}");
        Console.WriteLine($"Computed transformation:{referenceTransformation}");
        checkBackPoints = new();
        for (int i = 0; i < visibleInitialReferencePoints.Count; i++)
        {
            checkBackPoints.Add(TransformPointComputeError(visibleInitialReferencePoints[i], referenceTransformation, visibleReferencePoints[i]));
        }

        Console.WriteLine($"Back projection\nmax error {checkBackPoints.Max()}\nmean error {checkBackPoints.Mean()}");
        if (checkBackPoints.Any(i => i > _precitionPoints))
        {
            Console.WriteLine($"Not valid points {checkBackPoints.Count(i => i > _precitionPoints)} from {visibleInitialReferencePoints.Count}");
        }

        Matrix<double> computedTransformation = referenceTransformation.Inverse() * adapterTransformation;
        Console.WriteLine($"General transformation from adapter to ref CS: {computedTransformation}");

        List<ModelCsPoint> transformedPoints = new();
        (adapterPoints, adapterPointsInRefCs) = _transformationFinder.SelectPoints(adapterPoints, adapterPointsInRefCs);
        checkBackPoints = new();
        for (int i = 0; i < adapterPoints.Count; i++)
        {
            transformedPoints.Add(ApplyTransformation(adapterPoints[i], computedTransformation));
            checkBackPoints.Add((transformedPoints[i] - adapterPointsInRefCs[i]).L2Norm());
        }

        Console.WriteLine($"Back projection\nmax error {checkBackPoints.Max()}\nmean error {checkBackPoints.Mean()}");
        if (checkBackPoints.Any(i => i > _precitionPoints))
        {
            int numberNotValidPoints = checkBackPoints.Count(i => i > _precitionPoints);
            Console.WriteLine($"Not valid points {numberNotValidPoints} from {adapterPoints.Count}");
            if (numberNotValidPoints / adapterPoints.Count > 0.1)
            {
                Assert.Fail();
            }
        }

        Assert.Pass();
    }

    [Test]
    [TestCase("Points/Reference_SC_self.txt", "Points/Adapter_SC_self.txt", "Points/Ref+adapt_SC_MI.txt", "Points/Adapter_SC_Ref.txt")]
    public void TestLoadAdapterAndReferenceBackTransformation(string referenceFileName, string adapterFileName, string measuredDataFileName, string adapterInRefCsFileName)
    {
        MarkPoint<ModelCsPoint>[] referenceMarks = LoadPointsFromFile<ModelCsPoint>(referenceFileName);
        MarkPoint<ModelCsPoint>[] adapterMarks = LoadPointsFromFile<ModelCsPoint>(adapterFileName);
        MarkPoint<ModelCsPoint>[] measuredMarks = LoadPointsFromFile<ModelCsPoint>(measuredDataFileName);
        MarkPoint<ModelCsPoint>[] adapterMarksInRefCs = LoadPointsFromFile<ModelCsPoint>(adapterInRefCsFileName);
        List<ModelCsPoint> adapterPointsInRefCs = ConvertToPoints(adapterMarksInRefCs);

        AdapterModel adapter = (AdapterModel)ModelSerializer.LoadAdapterFromFile(adapterFileName);
        ReferenceModel reference = (ReferenceModel)ModelSerializer.LoadReferenceFromFile(referenceFileName);
        List<Model> objects = new()
        {
            adapter, reference
        };
        ModelsClassifier classifier = new(objects);
        List<ClassificationResult> measuredClassifiedResult = classifier.Classify(measuredMarks);
        Assert.That(measuredClassifiedResult, Has.Count.EqualTo(2), "Two models should be found");
        foreach(ClassificationResult result in measuredClassifiedResult)
        {
            Console.WriteLine($"{result.MeasuredObject.ModelName} mean fitting error: {result.MeanSquaredFittingError}");
        }
        Referencing referencing = new(reference, 0.3);
        ReferencingResult referencingResult = referencing.EstimateTransformations(measuredClassifiedResult);
        Assert.That(referencingResult.MeasuredObjects, Has.Count.EqualTo(1), "One model should be found");
        Console.WriteLine($"Referencing error: {referencingResult.ReferencingError}");
        MovableObject foundModel = referencingResult.MeasuredObjects[0];
        List<MarkPoint<ModelCsPoint>> transformedModelMarks = foundModel.GetTransformedModelPoints();
        List<ModelCsPoint> transformedModelPoints = transformedModelMarks.Select(point => point.Point).ToList();
        (List<ModelCsPoint> visibleTransformedPoints, List<ModelCsPoint> visibleMeasuredDataPoints)
            = _transformationFinder.SelectPoints<ModelCsPoint>(transformedModelPoints, adapterPointsInRefCs);
        List<double> checkBackPoints = new();
        for (int i = 0; i < visibleTransformedPoints.Count; i++)
        {
            checkBackPoints.Add((visibleTransformedPoints[i] - visibleMeasuredDataPoints[i]).L2Norm());
            Console.WriteLine($"Back projection error: {checkBackPoints[i]}");
        }

        Console.WriteLine($"Back projection\nmax error {checkBackPoints.Max()}\nmean error {checkBackPoints.Mean()}");
        if (checkBackPoints.Any(i => i > _precitionPoints))
        {
            int numberNotValidPoints = checkBackPoints.Count(i => i > _precitionPoints);
            Console.WriteLine($"Not valid points {numberNotValidPoints} from {visibleTransformedPoints.Count}");
            if (numberNotValidPoints / visibleTransformedPoints.Count > 0.1)
            {
                Assert.Fail();
            }
        }

        Assert.Pass();
    }
}