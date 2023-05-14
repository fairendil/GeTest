using DigitalAssembly.Photogrammetry;
using DigitalAssembly.Photogrammetry.Geometry.CoordinateSystems;
using Emgu.CV;
using Emgu.CV.Aruco;
using Emgu.CV.CvEnum;
using Emgu.CV.Features2D;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;

namespace DigitalAssembly.GoldenEye.Detector;

public static class OpenCvMarkDetector
{
    private static bool _saveImage = false;
    private static bool _saveCountourIndex = true;
    //private static readonly CudaHoughCirclesDetector _cudaHough = new(0.5f, 20, 60, 1, 9, 15);

    static void CreateBoard()
    {
        Dictionary dictionary = new Dictionary(Dictionary.PredefinedDictionaryName.Dict4X4_250);
        CharucoBoard board = new(5, 7, 0.04f, 0.02f, dictionary);
        Mat boardImage = new();
        //board.Draw(new Size(3508, 210), boardImage);
        CvInvoke.Imwrite("BoardImage.jpg", boardImage);
    }



    /// <summary>
    /// Simple uncoded circle marks detection method for images with Python OpenCV.
    /// </summary>
    /// <param name="imagePath"></param>
    /// <returns></returns>
    /// <exception cref="ImageBitmapNotValidException"></exception>
    public static List<MarkPoint<PixelCsPoint>> LoadAndDetect(string imagePath)
    {
        //CreateBoard();
        // Параметр, регулирующий степень увеличения изображения
        int scaleImageParameter = 2;
        Mat image = null;
        try
        {
            image = CvInvoke.Imread(imagePath, ImreadModes.Color);
        }
        catch (Exception ex)
        {
            throw new ImageBitmapNotValidException($"Unable ti load image from file: {ex}");
        }
        if (image == null)
        {
            throw new ImageBitmapNotValidException("Unable to load image from file");
        }

        Dictionary dictionary = new Dictionary(Dictionary.PredefinedDictionaryName.Dict4X4_50);
        VectorOfInt _allIds = new VectorOfInt();
        VectorOfVectorOfPointF _allCorners = new VectorOfVectorOfPointF();
        VectorOfInt _markerCounterPerFrame = new VectorOfInt();

        VectorOfMat corners_all = new VectorOfMat(5);
        VectorOfVectorOfPointF corners = new VectorOfVectorOfPointF();
        VectorOfInt ids = new VectorOfInt();
        VectorOfVectorOfPointF cornersDimond = new VectorOfVectorOfPointF();
        VectorOfInt idsDimond = new VectorOfInt();

        DetectorParameters Parameters = DetectorParameters.GetDefault();

        VectorOfVectorOfPointF Rejected = new VectorOfVectorOfPointF();

        for (int i = 7; i < 33; i++)
        {
            string imagesLocation = "_DigitalAssembly\\GoldenEye\\InitialData\\test_images\\charuco\\";
            string folderName = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), imagesLocation);

            //string leftPointsImageFile = Path.Join(folderName, $"{imagePairIndex}_00_tableOnly.bmp");
            string imagePairIndex = string.Format("{0:00}", i);
            string leftPointsImageFile = Path.Join(folderName, $"{imagePairIndex}_01.bmp");
            image = CvInvoke.Imread(leftPointsImageFile, ImreadModes.Color);
            ArucoInvoke.DetectMarkers(image, dictionary, corners, ids, Parameters, Rejected);
            _allCorners.Push(corners);
            _allIds.Push(ids);
            _markerCounterPerFrame.Push(new int[] { corners.Size });
        }

        Mat boardImage = new();
        image.CopyTo(boardImage);
        //ArucoInvoke.DrawDetectedCornersCharuco(boardImage, corners, ids, new MCvScalar(0, 100, 100));
        var path = Path.GetFullPath(imagePath);

        CvInvoke.Imwrite(Path.Combine(path, "DrawDetectedCornersCharuco.jpg"), boardImage);

        CharucoBoard charucoBoard = new(10, 8, 200, 140, dictionary);
        Mat cameraMatrix = new();
        using Mat distortionCoeff = new Mat();
        using Mat rotationVector = new Mat();
        using Mat translationVector = new Mat();
        using Mat stdDeviationsIntrinsics = new Mat();
        using Mat stdDeviationsExtrinsics = new Mat();
        using Mat perViewErrors = new Mat();

        MCvTermCriteria termCrit = new MCvTermCriteria(0.1e-5);
        CalibType calibType = CalibType.Default
                                 //| CalibType.FixAspectRatio
                                 //| CalibType.FixFocalLength
                                 //| CalibType.RationalModel
                                 | CalibType.ThinPrismModel
                                 | CalibType.TiltedModel
                                 ;

        double repError = ArucoInvoke.CalibrateCameraAruco(_allCorners,
                                                           _allIds,
                                                           _markerCounterPerFrame,
                                                           charucoBoard,
                                                           image.Size,
                                                           cameraMatrix,
                                                           distortionCoeff,
                                                           rotationVector,
                                                           translationVector,
                                                           stdDeviationsIntrinsics,
                                                           stdDeviationsExtrinsics,
                                                           perViewErrors,
                                                           calibType,
                                                           new MCvTermCriteria(100));

        Mat imgGrayscale = new Mat(image.Size * scaleImageParameter, DepthType.Cv8U, 1);
        Mat imgBrighter = new Mat(image.Size * scaleImageParameter, DepthType.Cv8U, 1);
        Mat imgThreshold = new Mat(image.Size * scaleImageParameter, DepthType.Cv8U, 1);

        // Предварительная обработка изображения для лучшего поиска контуров
        CvInvoke.CvtColor(image, imgGrayscale, ColorConversion.Bgr2Gray);
        // Важно: изображение увеличивается в 3 раза
        CvInvoke.Resize(imgGrayscale, imgGrayscale, new Size(imgGrayscale.Width * scaleImageParameter, imgGrayscale.Height * scaleImageParameter), interpolation: Inter.Lanczos4);
        CvInvoke.ConvertScaleAbs(imgGrayscale, imgBrighter, 1.7, 10);
        CvInvoke.GaussianBlur(imgBrighter, imgBrighter, new Size(3, 3), 2);
        CvInvoke.Threshold(imgBrighter, imgThreshold, 150, 255, ThresholdType.Otsu | ThresholdType.Binary);
        string name1 = Path.GetFileName(imagePath);
        imgThreshold.Save($"1\\{name1}_CountourIndex.bmp");
        // Найденные контура на увеличенном изображенииa
        VectorOfVectorOfPoint contours = new();
        //CvInvoke.FindContours(imgThreshold, contours, hierarchy: null, RetrType.List, ChainApproxMethod.LinkRuns);

        CircleF[] circles = CvInvoke.HoughCircles(imgBrighter, HoughModes.GradientAlt, 2.5, 160, 5, 0.6, 10, 20);

        SIFT simpleBlobDetector = new SIFT();

        MKeyPoint[] arrayOfKeyPoint = simpleBlobDetector.Detect(image);
        VectorOfKeyPoint vectorOfKeyPoint = new VectorOfKeyPoint(arrayOfKeyPoint);

        Features2DToolbox.DrawKeypoints(image, vectorOfKeyPoint, image, new Bgr(0, 0, 200));
        List<MarkPoint<PixelCsPoint>> result = new();

        for (int i = 0; i < circles.Count(); i++)
        {
            CircleF circle = circles[i];
            if (circle.Radius is < 3 or > 20)
            {
                // Отбрасываются слишком маленькие и слишком длинные контура - это не могут быть метки
                continue;
            }

            // Скалирование найденного центра вписанного эллипса на параметр, заданный в начале
            PixelCsPoint coordinate = new PixelCsPoint(circle.Center.X / scaleImageParameter, circle.Center.Y / scaleImageParameter);
            result.Add(MarkPoint<PixelCsPoint>.FromCode(0, MarkCodeType.Uncoded, coordinate));
            if (_saveCountourIndex)
            {
                CvInvoke.Circle(image, new Point((int)(circle.Center.X / scaleImageParameter), (int)(circle.Center.Y / scaleImageParameter)), (int)(circle.Radius / scaleImageParameter), new MCvScalar(235, 230, 30), 3, LineType.Filled);
                CvInvoke.DrawMarker(image, new Point((int)coordinate.X, (int)coordinate.Y), new MCvScalar(200, 200, 0), MarkerTypes.Cross);
                CvInvoke.PutText(image, (result.Count - 1).ToString() + " (" + circle.Radius.ToString() + @",\n" + coordinate.X.ToString() + @",\n" + coordinate.Y.ToString() + ")", new Point((int)coordinate.X, (int)coordinate.Y), FontFace.HersheyPlain, 1, new MCvScalar(200, 200, 0));
            }
        }

        imgGrayscale.Dispose();
        imgBrighter.Dispose();
        if (_saveImage)
        {
            CvInvoke.CvtColor(imgThreshold, imgThreshold, ColorConversion.Gray2Bgr);
            CvInvoke.DrawContours(imgThreshold, contours, -1, new MCvScalar(0, 0, 255), 5);
            imgThreshold.Save($"{DateTime.Now:HH-mm-ss-fff}.bmp");
        }

        imgThreshold.Dispose();
        if (_saveCountourIndex)
        {
            string name = Path.GetFileName(imagePath);
            image.Save($"1\\{name}_CountourIndex.bmp");
        }

        image.Dispose();
        return result;
    }
    public static List<MarkPoint<PixelCsPoint>> LoadAndDetectCentrsOnly(string imagePath)
    {
        // Параметр, регулирующий степень увеличения изображения
        int scaleImageParameter = 3;
        var name = Path.GetFileName(imagePath);
        Mat image = null;
        try
        {
            image = CvInvoke.Imread(imagePath, ImreadModes.Color);
        }
        catch (Exception ex)
        {
            throw new ImageBitmapNotValidException($"Unable ti load image from file: {ex}");
        }
        if (image == null)
        {
            throw new ImageBitmapNotValidException("Unable to load image from file");
        }

        Mat imgGrayscale = new Mat(image.Size, DepthType.Cv8U, 1);
        Mat imgBrighter = new Mat(image.Size, DepthType.Cv8U, 1);
        Mat imgThreshold = new Mat(image.Size, DepthType.Cv8U, 1);

        // Предварительная обработка изображения для лучшего поиска контуров
        CvInvoke.CvtColor(image, imgGrayscale, ColorConversion.Bgr2Gray);
        // Важно: изображение увеличивается в 3 раза
        CvInvoke.Resize(imgGrayscale, imgGrayscale, new Size(imgGrayscale.Width * scaleImageParameter, imgGrayscale.Height * scaleImageParameter), interpolation: Inter.Lanczos4);
        CvInvoke.Resize(image, image, new Size(image.Width * scaleImageParameter, image.Height * scaleImageParameter), interpolation: Inter.Lanczos4);
        CvInvoke.ConvertScaleAbs(imgGrayscale, imgBrighter, 1.5, 110);
        CvInvoke.ConvertScaleAbs(imgGrayscale, imgThreshold, 0.5, 10);
        var cicle = CvInvoke.GetStructuringElement(ElementShape.Ellipse, new Size(3, 3), new Point(1, 1));
        CvInvoke.MorphologyEx(imgBrighter, imgBrighter, MorphOp.Close, cicle, new Point(-1, -1), 1, BorderType.Default, CvInvoke.MorphologyDefaultBorderValue);

        CvInvoke.GaussianBlur(imgBrighter, imgBrighter, new Size(13, 13), 12);
        CvInvoke.Threshold(imgBrighter, imgBrighter, 40, 250, ThresholdType.Otsu);

        //var cicle = CvInvoke.GetStructuringElement(ElementShape.Ellipse, new Size(5, 5), new Point(-1, -1));
        CvInvoke.MorphologyEx(imgBrighter, imgBrighter, MorphOp.Open, cicle, new Point(-1, -1), 1, BorderType.Default, CvInvoke.MorphologyDefaultBorderValue);

        CircleF[] circles = CvInvoke.HoughCircles(imgBrighter, HoughModes.GradientAlt, 1, 60, 333, 0.8, 15, 35);
        SimpleBlobDetector simpleBlobDetector = new SimpleBlobDetector(new SimpleBlobDetectorParams()
        {
            FilterByCircularity = true,
            FilterByArea = true,
            MinCircularity = 0.1f,
            MaxCircularity = 100.0f,
            MinArea = 1,
            MaxArea = 5000
        });
        using VectorOfKeyPoint vectorOfKeyPoint = new VectorOfKeyPoint();
        simpleBlobDetector.DetectRaw(image, vectorOfKeyPoint);
        Features2DToolbox.DrawKeypoints(image, vectorOfKeyPoint, image, new Bgr(0, 0, 200));
        List<MarkPoint<PixelCsPoint>> result = new();

        for (int i = 0; i < circles.Length; i++)
        {
            var circle = circles[i];
            CvInvoke.Circle(image, new Point((int)circle.Center.X, (int)circle.Center.Y), (int)circle.Radius, new MCvScalar(235, 230, 30), 3, LineType.Filled);

            // Скалирование найденного центра вписанного эллипса на параметр, заданный в начале
            PixelCsPoint coordinate = new PixelCsPoint(circle.Center.X / scaleImageParameter, circle.Center.Y / scaleImageParameter);
            if (_saveCountourIndex)
            {
                CvInvoke.PutText(image, (result.Count - 1).ToString(), new Point((int)circle.Center.X, (int)circle.Center.Y), FontFace.HersheySimplex, 2, new MCvScalar(200, 200, 200));
            }
            result.Add(MarkPoint<PixelCsPoint>.FromCode(0, MarkCodeType.Uncoded, coordinate));
        }

        if (_saveCountourIndex)
        {
            name = Path.GetFileName(imagePath);
            image.Save($"1\\{name}_Circles.bmp");
        }
        imgBrighter.Dispose();
        imgThreshold.Dispose();
        imgGrayscale.Dispose();
        image.Dispose();
        return result;
    }

    public static void TestMatching()
    {
        string imagePath = "1//1.bmp";
        var name = Path.GetFileName(imagePath);
        Mat modelImage = CvInvoke.Imread(imagePath, ImreadModes.Color);
        imagePath = "1//2.bmp";
        Mat obervedImage = CvInvoke.Imread(imagePath, ImreadModes.Color);
        VectorOfVectorOfDMatch matches = new();
        FindMatch(modelImage, obervedImage, out long matchTime,
                                 out VectorOfKeyPoint modelKeyPoints,
                                 out VectorOfKeyPoint observedKeyPoints,
                                 matches,
                                 out Mat mask,
                                 out Mat homography);
        if (_saveCountourIndex)
        {
            var coll = new Bgr(0, 200, 200);
            MCvScalar color2 = coll.MCvScalar;
            using Mat inputArrayModel = modelImage.GetInputArray().GetMat();
            using Mat inputOutputArrayModel = modelImage.GetInputOutputArray().GetMat();
            Features2DToolbox.DrawKeypoints(inputArrayModel,
                                            modelKeyPoints,
                                            inputOutputArrayModel,
                                            coll,
                                            Features2DToolbox.KeypointDrawType.DrawRichKeypoints);

            //Features2DToolbox.DrawKeypoints(modelImage, modelKeyPoints, modelKeyPoints, new Bgr(0, 200, 200), Features2DToolbox.KeypointDrawType.DrawRichKeypoints);
            Mat inputOutputObserved = obervedImage.GetInputOutputArray().GetMat();
            Mat inputArrayObserved = obervedImage.GetInputArray().GetMat();
            Features2DToolbox.DrawKeypoints(inputArrayObserved,
                                            observedKeyPoints,
                                            inputOutputObserved,
                                            coll,
                                            Features2DToolbox.KeypointDrawType.Default);
            inputOutputArrayModel.Save($"1\\!1modelImage.bmp");
            inputOutputObserved.Save($"1\\!1observed.bmp");
        }
    }

    public static void FindMatch(Mat modelImage,
                                 Mat observedImage,
                                 out long matchTime,
                                 out VectorOfKeyPoint modelKeyPoints,
                                 out VectorOfKeyPoint observedKeyPoints,
                                 VectorOfVectorOfDMatch matches,
                                 out Mat mask,
                                 out Mat homography)
    {
        int k = 2;
        double uniquenessThreshold = 0.05;
        double hessianThresh = 300;

        Stopwatch watch;
        homography = null;

        modelKeyPoints = new VectorOfKeyPoint();
        observedKeyPoints = new VectorOfKeyPoint();

        using (UMat uModelImage = modelImage.GetUMat(AccessType.Read))
        using (UMat uObservedImage = observedImage.GetUMat(AccessType.Read))
        {
            ORB orbCPU = new ORB();
            //extract features from the object image
            UMat modelDescriptors = new UMat();
            orbCPU.DetectAndCompute(uModelImage, null, modelKeyPoints, modelDescriptors, false);

            watch = Stopwatch.StartNew();

            // extract features from the observed image
            UMat observedDescriptors = new UMat();
            orbCPU.DetectAndCompute(uObservedImage, null, observedKeyPoints, observedDescriptors, false);
            BFMatcher matcher = new BFMatcher(DistanceType.L2);
            matcher.Add(modelDescriptors);

            matcher.KnnMatch(observedDescriptors, matches, k, null);
            mask = new Mat(matches.Size, 1, DepthType.Cv8U, 1);
            mask.SetTo(new MCvScalar(255));
            Features2DToolbox.VoteForUniqueness(matches, uniquenessThreshold, mask);

            int nonZeroCount = CvInvoke.CountNonZero(mask);
            if (nonZeroCount >= 4)
            {
                nonZeroCount = Features2DToolbox.VoteForSizeAndOrientation(modelKeyPoints, observedKeyPoints,
                   matches, mask, 1.5, 20);
                if (nonZeroCount >= 4)
                    homography = Features2DToolbox.GetHomographyMatrixFromMatchedFeatures(modelKeyPoints,
                       observedKeyPoints, matches, mask, 2);
            }
        }

        matchTime = watch.ElapsedMilliseconds;
    }
}