using DigitalAssembly.GoldenEye.Classifier;
using DigitalAssembly.GoldenEye.DB;
using DigitalAssembly.GoldenEye.Detector;
using DigitalAssembly.GoldenEye.Serializers;
using DigitalAssembly.GoldenEye.Tracking;
using DigitalAssembly.Math.Common.Interfaces;
using DigitalAssembly.Photogrammetry;
using DigitalAssembly.Photogrammetry.Calibration;
using DigitalAssembly.Photogrammetry.Camera;
using DigitalAssembly.Photogrammetry.Geometry.CoordinateSystems;
using DigitalAssembly.Photogrammetry.Serializers;
using DigitalAssembly.Photogrammetry.Stereo;
using System.Data;

namespace DigitalAssembly.GoldenEye.Service;

internal class Program
{
    protected Program()
    {
    }

    private static void Main()
    {
        string projectDir = "SystemParameters-MI";

        Project project = LoadFromDirectory(projectDir);
        project = Project.FromProjectAndMyu(project, 1.002097);
        StereoSystemGeometry geometry = StereoSystemGeometry.FromProject(project);
        EnumeratePoints<ModelCsPoint> enumerator = new();

        //CalibrationMethod.ResetReprojectionThreshold();
        //OpenCvMarkDetector.TestMatching();
        string imagesLocation = "_DigitalAssembly\\GoldenEye\\InitialData\\test_images\\charuco\\";
        string folderName = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), imagesLocation);
        string imagePairIndex = "01";

        //string leftPointsImageFile = Path.Join(folderName, $"{imagePairIndex}_00_tableOnly.bmp");
        string leftPointsImageFile = Path.Join(folderName, $"{imagePairIndex}_01.bmp");
        string rightPointsImageFile = Path.Join(folderName, $"{imagePairIndex}_01.bmp");

        List<MarkPoint<PixelCsPoint>> leftPoints = OpenCvMarkDetector.LoadAndDetect(leftPointsImageFile);
        List<MarkPoint<PixelCsPoint>> rightPoints = new(); // OpenCvMarkDetector.LoadAndDetectCentrsOnly(rightPointsImageFile);
        List<MarkPoint<ModelCsPoint>> points = new(); // geometry.Compute3DCoords2(leftPoints, rightPoints).ToList();
        List<MarkPoint<PixelCsPoint>> rightComputedPoints = new();//rightPoints.Where(p => p.MarkCode.Code >= 0).OrderBy(p => p.MarkCode.Code).ToList();
        List<MarkPoint<PixelCsPoint>> leftComputedPoints = new();//leftPoints.Where(p => p.MarkCode.Code >= 0).ToList();

        string referenceTabel = "Points/ReferenceTableInOrder.txt";
        MarkPoint<ModelCsPoint>[] table = ModelSerializer.LoadPointsFromFile<ModelCsPoint>(referenceTabel);

        foreach (var point in points)
        {
            double x = point.Point.X;
            double y = -point.Point.Y;
            double z = -point.Point.Z;
            point.Point.X = x;
            point.Point.Y = y;
            point.Point.Z = z;
        }

        int startValue = 1;
        int endValue = 15;
        int countValue = endValue - startValue;

        int[] imageIndexs = new int[] { 9, 11 };

        //First extrinsic parameters approximation
        for (int i = startValue; i <= endValue; i += 1)
        {
            if (imageIndexs.Contains(i))
            {
                continue;
            }

            //geometry = StereoSystemGeometry.FromProject(project);
            imagePairIndex = string.Format("{0:0000}", i);
            Console.WriteLine($"Progress: {(i - startValue) * 100.0 / countValue}%");
            leftPoints = OpenCvMarkDetector.LoadAndDetect(Path.Join(folderName, $"{imagePairIndex}_00.bmp"));
            rightPoints = OpenCvMarkDetector.LoadAndDetect(Path.Join(folderName, $"{imagePairIndex}_01.bmp"));
            if (leftPoints.Count < 5 || rightPoints.Count < 5)
            {
                continue;
            }

            points = geometry.Compute3DCoords(leftPoints, rightPoints, true).ToList();
            if (points.Count < 5)
            {
                continue;
            }

            rightComputedPoints = rightPoints.Where(p => p.MarkCode.Code >= 0).OrderBy(p => p.MarkCode.Code).ToList();
            leftComputedPoints = leftPoints.Where(p => p.MarkCode.Code >= 0).ToList();

            //int index = 0;
            foreach (var point in points)
            {
                //MarkPoint<PixelCsPoint> leftPoint = leftComputedPoints.ElementAt(index);
                //MarkPoint<PixelCsPoint> rightPoint = rightComputedPoints.ElementAt(index);
                //using StreamWriter corrdinatesFile = File.AppendText(Path.Join(folderName, $"corrdinates.txt"));

                //corrdinatesFile.WriteLine($"{point.Point.X} {point.Point.Y} {point.Point.Z} {leftPoint.Point.X} {leftPoint.Point.Y} {rightPoint.Point.X} {rightPoint.Point.Y} ");
                double x = point.Point.X;
                double y = -point.Point.Y;
                double z = -point.Point.Z;
                point.Point.X = x;
                point.Point.Y = y;
                point.Point.Z = z;
                //index++;
            }

            geometry = geometry.Calibrate(new CalibrationMethod(points, leftComputedPoints, rightComputedPoints, imagePairIndex, i));
        }

        //Scalebar calibration
        //Planar scalebar
        //X 
        //Y 
        //XY
        //X-Y
        folderName = "c:\\test_images\\scalebar";

        double myu = project.Myu;
        startValue = 0;
        endValue = 58;
        countValue = endValue - startValue;
        double dephtScaleBarLenght = 959.76;
        double planarScaleBarLenght = 885.816;
        double scaleBarLenghtErrorTreshhold = (dephtScaleBarLenght - planarScaleBarLenght) / 2;

        double[] distances = new double[125];

        for (int i = startValue; i <= endValue; i += 1)
        {
            project = Project.FromProjectAndMyu(project, myu);
            geometry = StereoSystemGeometry.FromProject(project);
            imagePairIndex = string.Format("{0:0000}", i);
            Console.WriteLine($"Progress: {(i - startValue) * 100.0 / countValue}%");
            leftPoints = OpenCvMarkDetector.LoadAndDetect(Path.Join(folderName, $"{imagePairIndex}_00.bmp"));
            rightPoints = OpenCvMarkDetector.LoadAndDetect(Path.Join(folderName, $"{imagePairIndex}_01.bmp"));
            points = geometry.Compute3DCoords(leftPoints, rightPoints).ToList();

            if (points.Count != 2)
            {
                continue;
            }

            var pointA = points[0].Point.Coordinate;
            var pointB = points[1].Point.Coordinate;
            double distance = (pointA - pointB).L2Norm();
            Console.WriteLine("______________________________");
            double scaleBarLenghtError = System.Math.Abs(distance - planarScaleBarLenght);
            if (scaleBarLenghtError > scaleBarLenghtErrorTreshhold)
            {
                continue;
            }

            //TODO проверить вариант расчета мю
            //возмжно для нашей геометрии стоит инвертировать деление и привести к 
            //Ближняя фотограметрия Страница 69 формула 2.50
            myu = planarScaleBarLenght / distance; //(myu + planarScaleBarLenght / distance) / 2;
            distances[i] = distance;
        }

        //Non-planar(depth) scalebar
        //Z
        //
        startValue = 59;
        endValue = 124;
        countValue = endValue - startValue;
        imageIndexs = new int[] { 93, 94, 96, 97, 98, 99, 100, 101, 102, 103 };
        for (int i = startValue; i <= endValue; i += 1)
        {
            if (imageIndexs.Contains(i))
            {
                continue;
            }

            project = Project.FromProjectAndMyu(project, myu);
            geometry = StereoSystemGeometry.FromProject(project);
            imagePairIndex = string.Format("{0:0000}", i);
            Console.WriteLine($"Progress: {(i - startValue) * 100.0 / countValue}%");
            leftPoints = OpenCvMarkDetector.LoadAndDetect(Path.Join(folderName, $"{imagePairIndex}_00.bmp"));
            rightPoints = OpenCvMarkDetector.LoadAndDetect(Path.Join(folderName, $"{imagePairIndex}_01.bmp"));
            points = geometry.Compute3DCoords(leftPoints, rightPoints, true).ToList();

            if (points.Count != 2)
            {
                continue;
            }

            var pointA = points[0].Point.Coordinate;
            var pointB = points[1].Point.Coordinate;
            double distance = (pointA - pointB).L2Norm();
            double scaleBarLenghtError = System.Math.Abs(distance - dephtScaleBarLenght);
            Console.WriteLine($"______________________________");
            if (scaleBarLenghtError > scaleBarLenghtErrorTreshhold)
            {
                continue;
            }

            myu = dephtScaleBarLenght / distance; //(myu + dephtScaleBarLenght / distance) / 2;
            distances[i] = distance;
        }

        project = Project.FromProjectAndMyu(project, myu);
        //Calculating error
        startValue = 0;
        endValue = 124;
        countValue = endValue - startValue;

        for (int i = startValue; i <= endValue; i += 1)
        {

            geometry = StereoSystemGeometry.FromProject(project);
            imagePairIndex = string.Format("{0:0000}", i);
            Console.WriteLine($"Progress: {(i - startValue) * 100.0 / countValue}%");
            leftPoints = OpenCvMarkDetector.LoadAndDetect(Path.Join(folderName, $"{imagePairIndex}_00.bmp"));
            rightPoints = OpenCvMarkDetector.LoadAndDetect(Path.Join(folderName, $"{imagePairIndex}_01.bmp"));
            points = geometry.Compute3DCoords(leftPoints, rightPoints, true).ToList();

            if (points.Count != 2)
            {
                continue;
            }

            var pointA = points[0].Point.Coordinate;
            var pointB = points[1].Point.Coordinate;
            double distance = (pointA - pointB).L2Norm();

            distances[i] = distance;

        }

        List<MarkPoint<ModelCsPoint>> enumerated = enumerator.Enumerate(points);
        foreach (MarkPoint<ModelCsPoint> point in enumerated)
        {
            Console.WriteLine(point);
        }

        string referenceFileName = "Points/Reference_SC_self.txt";
        string adapterFileName = "Points/Adapter_SC_self.txt";
        AdapterModel adapter = (AdapterModel)ModelSerializer.LoadAdapterFromFile(adapterFileName);
        ReferenceModel reference = (ReferenceModel)ModelSerializer.LoadReferenceFromFile(referenceFileName);
        List<Model> objects = new()
        {
            adapter, reference
        };
        ModelsClassifier classifier = new(objects);
        var result = classifier.Classify(points.ToArray());
        ;
    }

    public static Project LoadFromDirectory(string directoryName)
    {
        CameraModel left = CameraModelSerializer.LoadCameraModel(directoryName, "Left");
        CameraModel right = CameraModelSerializer.LoadCameraModel(directoryName, "Right");

        //First extrinsic parameters approximation
        //Plate configuration
        return new Project(left, right, new StereoGeometryParameters(2 * 1e3, 4.5 * 1e3, 0.00985, 1e-6), 1.0003419125007675);
        //Scalebar calibration Icon
        //return new Project(left, right, new StereoGeometryParameters(2 * 1e3, 4.5 * 1e3, 0.0985, 1e-6), 1.00125095249899);
        //Scalebar calibration OpenCV
        //return new Project(left, right, new StereoGeometryParameters(2 * 1e3, 4.5 * 1e3, 0.0985, 1e-6), 1.0003419125007675);
    }

    public static (List<MarkPoint<T>> left, List<MarkPoint<T>> right) MapMarkPoints<T>(MarkPoint<T>[] leftMarks, MarkPoint<T>[] rightMarks)
        where T : IPoint
    {
        List<MarkPoint<T>> left = leftMarks.Where(i => i.HasCode).OrderBy(i => i.MarkCode.Code).ToList();
        List<MarkPoint<T>> right = rightMarks.Where(i => i.HasCode).OrderBy(i => i.MarkCode.Code).ToList();

        List<int> intersectedCodes = left.Select(i => i.MarkCode.Code).Intersect(right.Select(j => j.MarkCode.Code)).ToList();

        List<MarkPoint<T>> leftResult = left.Where(i => intersectedCodes.Contains(i.MarkCode.Code)).ToList();
        List<MarkPoint<T>> rightResult = right.Where(i => intersectedCodes.Contains(i.MarkCode.Code)).ToList();

        return (leftResult, rightResult);
    }
}