using DigitalAssembly.GoldenEye.Detector;
using DigitalAssembly.Math.Common;
using DigitalAssembly.Photogrammetry;
using DigitalAssembly.Photogrammetry.Camera;
using DigitalAssembly.Photogrammetry.Geometry.CoordinateSystems;
using DigitalAssembly.Photogrammetry.Serializers;
using DigitalAssembly.Photogrammetry.Stereo.Geometry;
using System.Text;
using System.Text.RegularExpressions;

namespace DigitalAssembly.GoldenEye.UnitTests.Epipolar;

public class EpipolarGeometryTest
{
    private EpipolarGeometry _epipolarGeometry;
    private CameraGeometry _cameraGeometryLeft;
    private CameraGeometry _cameraGeometryRight;

    [SetUp]
    public void Setup()
    {
        string projectDir = @"SystemParameters-MI";
        CameraModel left = CameraModelSerializer.LoadCameraModel(projectDir, "Left");
        CameraModel right = CameraModelSerializer.LoadCameraModel(projectDir, "Right");
        _cameraGeometryLeft = new CameraGeometry(left);
        _cameraGeometryRight = new CameraGeometry(right);
        StereoGeometry stereoGeometry = new(left, right);
        // TODO: fixate stereo params or use static DigitalAssembly.GoldenEye.Service.Globals???
        StereoGeometryParameters stereoParameters = new(2 * 1e3, 4.5 * 1e3, 0.00985, 1e-6);
        _epipolarGeometry = EpipolarGeometry.Init(stereoGeometry, stereoParameters);
    }

    [Test]
    [TestCase("TestImagePair", "0053")]
    [TestCase("TestImagePair", "0225")]
    public void TestEpipolarGeometry(string folderName, string imagePairIndex)
    {
        // TODO: Change Console.WriteLine to graceful logger in tests
        string leftPointsImageFile = Path.Join(folderName, $"{imagePairIndex}_00_cleanedMarks.bmp");
        string rightPointsImageFile = Path.Join(folderName, $"{imagePairIndex}_01_cleanedMarks.bmp");
        string pairsIndexesFile = Path.Join(folderName, $"{imagePairIndex}_cleaned_PairsIndexes.txt");
        List<(int LeftIndex, int RightIndex)> validateIndexes = LoadPixelPointsPairsIndexes(pairsIndexesFile, @"\s+");
        validateIndexes = validateIndexes.OrderBy(i => i.LeftIndex).ToList();

        List<MarkPoint<PixelCsPoint>> leftPoints = OpenCvMarkDetector.LoadAndDetect(leftPointsImageFile);
        List<MarkPoint<PixelCsPoint>> rightPoints = OpenCvMarkDetector.LoadAndDetect(rightPointsImageFile);

        List<MarkPoint<PixelCsPoint>> leftSorted = SortPointsByX(leftPoints);
        List<MarkPoint<PixelCsPoint>> rightSorted = SortPointsByX(rightPoints);

        List<MarkPoint<UndistortedPictureCsPoint>> leftUndistorted = _cameraGeometryLeft.Undistort(leftSorted).ToList();
        List<MarkPoint<UndistortedPictureCsPoint>> rightUndistorted = _cameraGeometryRight.Undistort(rightSorted).ToList();
        Dictionary<int, MarkPoint<HomogeneousPictureCsPoint>> leftHomo = Enumerable.Range(0, leftUndistorted.Count)
            .ToDictionary(i => i, i => leftUndistorted[i].UpdateCoordinate(leftUndistorted[i].Point.ToHomogeneousCoordinates()));
        Dictionary<int, MarkPoint<HomogeneousPictureCsPoint>> rightHomo = Enumerable.Range(0, rightUndistorted.Count)
            .ToDictionary(i => i, i => rightUndistorted[i].UpdateCoordinate(rightUndistorted[i].Point.ToHomogeneousCoordinates()));

        Dictionary<int, List<int>> pairsForLeft = new();
        for (int i = 0; i < leftHomo.Count; ++i)
        {
            (List<int> tmp, double bestDistance) = _epipolarGeometry.FindPairsIndexes(leftHomo[i].Point, rightHomo.Values.Select(t => t.Point).ToList(), chosenPointIsLeft: true);
            if (tmp.Count > 0)
            {
                pairsForLeft.Add(i, tmp);
                Console.WriteLine($"For left point {i} best distance for right point {tmp[0]}: {bestDistance}");
            }
        }

        Dictionary<int, List<int>> pairsForRight = new();
        for (int i = 0; i < rightHomo.Count; ++i)
        {
            (List<int> tmp, double bestDistance) = _epipolarGeometry.FindPairsIndexes(rightHomo[i].Point, leftHomo.Values.Select(t => t.Point).ToList(), chosenPointIsLeft: false);
            if (tmp.Count > 0)
            {
                pairsForRight.Add(i, tmp);
                Console.WriteLine($"For right point {i} best distance for left point {tmp[0]}: {bestDistance}");
            }
        }

        List<(int left, int right)> pairs = new();
        foreach (KeyValuePair<int, List<int>> pair in pairsForLeft)
        {
            List<int> foundRight = pair.Value;
            foreach (int index in foundRight)
            {
                if (pairsForRight.ContainsKey(index))
                {
                    int indexRight = index;
                    List<int> foundLeft = pairsForRight[indexRight];
                    if (foundLeft.Contains(pair.Key))
                    {
                        pairs.Add((pair.Key, indexRight));
                        pairsForRight.Remove(indexRight);
                        Console.WriteLine($"left #{pair.Key} : right #{indexRight}");
                        break;
                    }
                }
            }
        }

        pairs = pairs.OrderBy(i => i.left).ToList();
        Assert.That(validateIndexes.Count, Is.EqualTo(pairs.Count));

        List<bool> result = new();
        for (int i = 0; i < pairs.Count; ++i)
        {
            (int leftIndex, int rightIndex) computed = pairs[i], validate = validateIndexes[i];
            if (computed.leftIndex == validate.leftIndex)
            {
                if (computed.rightIndex == validate.rightIndex)
                {
                    result.Add(true);
                    continue;
                }
                result.Add(false);
                continue;
            }

            throw new Exception($"Left value '{computed.leftIndex}' from computed not match validate value '{validate.leftIndex}'");
        }

        if (!result.Contains(false))
        {
            Assert.Pass();
            return;
        }

        StringBuilder bld = new();
        for (int i = 0; i < result.Count; ++i)
        {
            if (!result[i])
            {
                bld.Append($"Point #{i} found incorrect pair\n");
            }
        }

        Console.WriteLine(bld);
        Assert.Fail();
    }

    private static List<MarkPoint<T>> SortPointsByX<T>(List<MarkPoint<T>> marks)
        where T : Point2D<T>
        => marks.OrderBy(i => i.Point.X).ToList();

    private static List<(int LeftIndex, int RightIndex)> LoadPixelPointsPairsIndexes(string fileName, string delimeterRegex)
    {
        List<(int LeftIndex, int RightIndex)> result = new();
        string[] lines = File.ReadAllLines(fileName);
        for (int i = 0; i < lines.Length; ++i)
        {
            string line = lines[i];
            string[] intStrings;
            try
            {
                intStrings = Regex.Split(line, delimeterRegex);
            }
            catch (Exception e)
            {
                throw new SerializerException($"Cannot split line '{i}': {e.Message}");
            }

            if (intStrings.Length != 2)
            {
                throw new SerializerException($"Number of int values in line '{i}' does not match 'intCount = {2}'");
            }

            List<int> values = new();
            foreach (string intString in intStrings)
            {
                if (!int.TryParse(intString, out int value))
                {
                    throw new SerializerException($"Value '{intString}' is not a int in line '{i}'");
                }
                values.Add(value);
            }

            result.Add((values[0], values[1]));
        }

        return result;
    }
}