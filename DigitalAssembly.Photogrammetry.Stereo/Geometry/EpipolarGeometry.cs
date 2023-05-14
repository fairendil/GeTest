using DigitalAssembly.Photogrammetry.Geometry.CoordinateSystems;
using DigitalAssembly.Photogrammetry.Stereo.Exceptions;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Spatial.Euclidean;
using MathNet.Spatial.Units;
using System.Text;
using static System.Math;

namespace DigitalAssembly.Photogrammetry.Stereo.Geometry;

/// <summary>
/// It works if left camera in ZERO POINT of coordinate system
/// </summary>
internal class EpipolarGeometry
{
    private readonly StereoGeometryParameters _globalParameters;
    private Matrix<double> _rotation;
    private Vector<double> _translation, _extrinsicTranslation;
    private Matrix<double> _intrisicLeft, _intrisicRight;
    private Matrix<double> _essentialMatrix, _fundamentalMatrix, _homographyMatrixNear, _homographyMatrixFar;
    private Matrix<double> _homographyMatrixNearInverse, _homographyMatrixFarInverse;
    private double _precision;
    private double _numberPointPairUncertainty;

    public StereoGeometryParameters GlobalParameters => _globalParameters;

    private EpipolarGeometry(StereoGeometryParameters geometryParameters)
    {
        _rotation = Matrix3D.RotationAroundXAxis(Angle.FromRadians(0));
        _translation = Vector<double>.Build.Dense(new double[] { 0, 0, 0 });
        _extrinsicTranslation = _translation;
        _intrisicLeft = Matrix<double>.Build.DenseDiagonal(3, 3, 1);
        _intrisicRight = Matrix<double>.Build.DenseDiagonal(3, 3, 1);
        _essentialMatrix = Matrix<double>.Build.DenseDiagonal(3, 3, 1);
        _fundamentalMatrix = Matrix<double>.Build.DenseDiagonal(3, 3, 1);
        _homographyMatrixFar = Matrix<double>.Build.DenseDiagonal(3, 3, 1);
        _homographyMatrixNear = Matrix<double>.Build.DenseDiagonal(3, 3, 1);
        _homographyMatrixFarInverse = Matrix<double>.Build.DenseDiagonal(3, 3, 1);
        _homographyMatrixNearInverse = Matrix<double>.Build.DenseDiagonal(3, 3, 1);
        _precision = 1e-5;
        _numberPointPairUncertainty = 1;
        _globalParameters = geometryParameters;
    }

    public static EpipolarGeometry Init(StereoGeometry stereo, StereoGeometryParameters geometryParameters)
    {
        EpipolarGeometry epipolarGeometry = new EpipolarGeometry(geometryParameters);
        try
        {
            epipolarGeometry._precision = geometryParameters.Precision;
            // Basic extrisic parameters
            epipolarGeometry._rotation = stereo.Rotation.right.Transpose();
            epipolarGeometry._translation = stereo.Translation.right;
            //TODO: возможно добвить stereo.Myu
            epipolarGeometry._extrinsicTranslation = -epipolarGeometry._rotation * epipolarGeometry._translation;

            // Intrisic parameters
            (Matrix<double> left, Matrix<double> right) = stereo.Intrisic;
            epipolarGeometry._intrisicLeft = left;
            epipolarGeometry._intrisicRight = right;
            Matrix<double> _intrisicLeftInverse = epipolarGeometry._intrisicLeft.Inverse();
            Matrix<double> _intrisicRightInverse = epipolarGeometry._intrisicRight.Inverse();
            // Essential matrix
            Vector<double> t = epipolarGeometry._extrinsicTranslation;
            epipolarGeometry._essentialMatrix = Matrix<double>.Build.DenseOfArray(new double[,] { { 0, -t[2], t[1] }, { t[2], 0, -t[0] }, { -t[1], t[0], 0 } }) * epipolarGeometry._rotation;
            // Additional matrices and vectors for fundamental matrix
            Vector<double> RotT = epipolarGeometry._rotation.TransposeThisAndMultiply(t);
            Matrix<double> RT = Matrix<double>.Build.DenseOfArray(new double[,] { { 0, -RotT[2], RotT[1] }, { RotT[2], 0, -RotT[0] }, { -RotT[1], RotT[0], 0 } });
            // Fundamental matrix
            epipolarGeometry._fundamentalMatrix = _intrisicRightInverse.TransposeThisAndMultiply(epipolarGeometry._rotation) * RT * _intrisicLeftInverse;
            // Additional matrices and vectors for homography
            Vector<double> zOne = Vector<double>.Build.DenseOfArray(new double[] { 0, 0, -1 });
            Vector<double> planeNormale = zOne + epipolarGeometry._rotation.TransposeThisAndMultiply(zOne);
            planeNormale /= planeNormale.L2Norm();

            Vector<double> planePoint = (epipolarGeometry._extrinsicTranslation / 2) - (planeNormale * geometryParameters.WorkingSpace.Nearest);
            double planeTranslation = planeNormale * planePoint;
            RT = t.ToRowMatrix().TransposeThisAndMultiply(planeNormale.ToRowMatrix());
            // Homography for nearest points from left to right
            epipolarGeometry._homographyMatrixNear = epipolarGeometry._intrisicRight * (epipolarGeometry._rotation - RT / planeTranslation) * _intrisicLeftInverse;
            RT = epipolarGeometry._translation.ToRowMatrix().TransposeThisAndMultiply(planeNormale.ToRowMatrix());
            // Homography for nearest points from right to left
            epipolarGeometry._homographyMatrixNearInverse = epipolarGeometry._intrisicLeft * (epipolarGeometry._rotation.Transpose() - RT / planeTranslation) * _intrisicRightInverse;

            planePoint = (epipolarGeometry._extrinsicTranslation / 2) - (planeNormale * geometryParameters.WorkingSpace.Farthest);
            planeTranslation = planeNormale * planePoint;
            RT = t.ToRowMatrix().TransposeThisAndMultiply(planeNormale.ToRowMatrix());
            // Homography for farthest points from left to right
            epipolarGeometry._homographyMatrixFar = epipolarGeometry._intrisicRight.Multiply(epipolarGeometry._rotation - RT / planeTranslation).Multiply(_intrisicLeftInverse);
            RT = epipolarGeometry._translation.ToRowMatrix().TransposeThisAndMultiply(planeNormale.ToRowMatrix());
            // Homography for farthest points from right to left
            epipolarGeometry._homographyMatrixFarInverse = epipolarGeometry._intrisicLeft.Multiply(epipolarGeometry._rotation.Transpose() - RT / planeTranslation).Multiply(_intrisicRightInverse);
        }
        catch (Exception ex)
        {
            throw new MathNotValidException("Some of calculation while building epipolar math matrices not valid", ex);
        }

        int maxPossiblePointNumber = 100;
        double imageArea = stereo.ImageArea.Left;
        epipolarGeometry._numberPointPairUncertainty = maxPossiblePointNumber * (maxPossiblePointNumber - 1) *
                                (2 * epipolarGeometry._globalParameters.EpipolarEpsilon * Abs(stereo.Right.IntrisicMatrix[0, 0]) *
                                 epipolarGeometry._translation.L2Norm() * (geometryParameters.WorkingSpace.Farthest - geometryParameters.WorkingSpace.Nearest)) /
                                (imageArea * geometryParameters.WorkingSpace.Farthest * geometryParameters.WorkingSpace.Nearest);

        bool[] statuses = epipolarGeometry.CheckMatricesValidy();
        List<int> unsuccIndexes = new();
        for (int i = 0; i < statuses.Length; ++i)
        {
            if (!statuses[i])
            {
                unsuccIndexes.Add(i);
            }
        }

        if (unsuccIndexes.Count > 0)
        {
            StringBuilder bld = new();
            for (int i = 0; i < unsuccIndexes.Count; ++i)
            {
                bld.Append($"{unsuccIndexes[i]},");
            }

            string str = bld.ToString();
            throw new MathNotValidException($"Matrices '{str}' in epipolar math not passes check");
        }

        return epipolarGeometry;
    }

    /// <summary>
    /// Unfinished. Check Essential, Fundamental, Nearest, Farthest homographies direct and reverse matrices. 
    /// </summary>
    /// <returns> Status indexes: 0 for essential matrix, 1 for fundamental matrix, 2 for direct nearest homography, 3 for direct farthest homography</returns>
    private bool[] CheckMatricesValidy()
    {
        bool[] result = new bool[4];
        for (int i = 0; i < 4; i++)
        {
            result[i] = false;
        }
        // Essential matrix analysis
        MathNet.Numerics.LinearAlgebra.Factorization.Svd<double> _svdDecomposition = _essentialMatrix.Svd(computeVectors: true);
        Vector<double> singularValues = _svdDecomposition.W.Diagonal();
        bool tmpStatus = (Abs(singularValues[0] - singularValues[1]) < _precision && Abs(singularValues[2]) < _precision) ||
                         (Abs(singularValues[1] - singularValues[2]) < _precision && Abs(singularValues[0]) < _precision) ||
                         (Abs(singularValues[0] - singularValues[2]) < _precision && Abs(singularValues[1]) < _precision);
        Matrix<double> essentialAnother = _intrisicRight.TransposeThisAndMultiply(_fundamentalMatrix) * _intrisicLeft;
        Matrix<double> tmp = essentialAnother - _essentialMatrix;

        if (tmpStatus && Abs(tmp.Enumerate().Max()) < _precision)
        {
            result[0] = true;
        }
        //Fundamental matrix analysis
        tmpStatus = _fundamentalMatrix.Rank() == 2;
        Vector<double> epipoleLeft = _intrisicLeft * _rotation.TransposeThisAndMultiply(_extrinsicTranslation);
        Vector<double> epipoleRight = _intrisicRight * _extrinsicTranslation;
        epipoleLeft /= epipoleLeft[2];
        epipoleRight /= epipoleRight[2];
        tmpStatus &= epipoleRight * _fundamentalMatrix * epipoleLeft < _precision;
        result[1] = tmpStatus;

        // Homography check
        tmp = _homographyMatrixNear.TransposeThisAndMultiply(_fundamentalMatrix) + _fundamentalMatrix.TransposeThisAndMultiply(_homographyMatrixNear);
        if (Abs(tmp.Enumerate().Max()) < _precision)
        {
            result[2] = true;
        }

        tmp = _homographyMatrixFar.TransposeThisAndMultiply(_fundamentalMatrix) + _fundamentalMatrix.TransposeThisAndMultiply(_homographyMatrixFar);
        if (Abs(tmp.Enumerate().Max()) < _precision)
        {
            result[3] = true;
        }

        return result;
    }

    private Vector<double> EpipoleLine(Vector<double> chosenPoint, bool chosenPointIsLeft)
    {
        Matrix<double> fundamental = _fundamentalMatrix;
        if (!chosenPointIsLeft)
            fundamental = fundamental.Transpose();
        Vector<double> epipoleLine = fundamental * chosenPoint;
        epipoleLine /= epipoleLine.SubVector(0, 2).L2Norm();
        return epipoleLine;
    }

    /// <summary>
    /// Finds all related points for chosen point in list of points from another camera.
    /// If you need to find related points from right picture to left pictire point, leftright parameter shouls be true. 
    /// If you need to find related points from left picture to right pictire point, leftright parameter shouls be false.
    /// </summary>
    /// <param name="chosenPoint"></param>
    /// <param name="possiblePairPoints"></param>
    /// <param name="chosenPointIsLeft"></param>
    /// <returns>Indexes of paired points (sorted by distance) from initial list of points and best distance from possiblePai to epopola line</returns>
    public (List<int> Indexes, double BestDistance) FindPairsIndexes(HomogeneousPictureCsPoint chosenPoint, List<HomogeneousPictureCsPoint> possiblePairPoints, bool chosenPointIsLeft, bool scaleBar = false)
    {
        Matrix<double> homographyNear = _homographyMatrixNear, homographyFar = _homographyMatrixFar;
        if (!chosenPointIsLeft)
        {
            homographyNear = _homographyMatrixNearInverse;
            homographyFar = _homographyMatrixFarInverse;
        }

        Vector<double> point = chosenPoint.Coordinate;
        Vector<double> epipoleLineSegmentStart = homographyFar * point,
                       epipoleLineSegmentEnd = homographyNear * point;
        // Нормирование координат точек, так как они выше представлены в обобщенных координатах
        epipoleLineSegmentStart /= epipoleLineSegmentStart[2];
        epipoleLineSegmentEnd /= epipoleLineSegmentEnd[2];

        Vector<double> vecEpipole = EpipoleLine(point, chosenPointIsLeft);
        Vector<double> epipoleLineNormale = (epipoleLineSegmentEnd - epipoleLineSegmentStart).SubVector(0, 2);
        Line2D line = new Line2D(
                                new Point2D(epipoleLineSegmentStart[0], epipoleLineSegmentStart[1]),
                                new Point2D(epipoleLineSegmentEnd[0], epipoleLineSegmentEnd[1]));

        List<(int index, double value)> distances = new();
        double minDistance = 0;
        for (int i = 0; i < possiblePairPoints.Count; i++)
        {
            Vector<double> pointRightCoords = possiblePairPoints[i].Coordinate;
            double distance = vecEpipole * pointRightCoords;
            minDistance = minDistance == 0 ? distance : minDistance > distance ? distance : minDistance;
            // TODO: Benchmark using MathNet.Spatial.Euclidean structures
            Point2D rightP = new Point2D(pointRightCoords[0], pointRightCoords[1]);
            Point2D nearP = line.ClosestPointTo(rightP, false);
            Vector<double> pp = Vector<double>.Build.DenseOfArray(new double[] { nearP.X, nearP.Y });
            Vector<double> parametersForPositionOnLine = (pp - epipoleLineSegmentStart.SubVector(0, 2)) / epipoleLineNormale;
            double parameterForPositionOnLineToStart = ((pp - epipoleLineSegmentStart.SubVector(0, 2)) / epipoleLineNormale)[0];
            double parameterForPositionOnLineToEnd = ((epipoleLineSegmentEnd.SubVector(0, 2) - pp) / epipoleLineNormale)[1];
            //if (Abs(distance) < _globalParameters.EpipolarEpsilon && )
            if ((Abs(distance) < _globalParameters.EpipolarEpsilon)
                &&
                ((!scaleBar && parametersForPositionOnLine[0] >= 0 && parametersForPositionOnLine[0] <= 1)
                 || (scaleBar && parameterForPositionOnLineToStart >= 0 && parameterForPositionOnLineToEnd <= 1))
                 )
            {
                distances.Add((i, Abs(distance)));
            }
        }

        IEnumerable<(int index, double value)> result = distances.OrderBy(i => i.value);
        List<int> indexes = result.Select(i => i.index).ToList();
        double bestDistance = -1;
        if (indexes.Count > 0)
        {
            bestDistance = result.First().value;
        }

        return (indexes, bestDistance);
    }

    public IEnumerable<MarkPointPair<HomogeneousPictureCsPoint>>
        PairMarks(IEnumerable<MarkPoint<UndistortedPictureCsPoint>> left, IEnumerable<MarkPoint<UndistortedPictureCsPoint>> right, bool scaleBar = false)
    {
        List<MarkPointPair<HomogeneousPictureCsPoint>> result = new();
        // Pairing by MarkCode
        List<HomogeneousPictureCsPoint> leftHomo = new(), rightHomo = new();
        IEnumerable<MarkCode> leftCodedPoints = left.Where(point => point.HasCode).Select(point => point.MarkCode);
        IEnumerable<MarkCode> rightCodedPoints = right.Where(point => point.HasCode).Select(point => point.MarkCode);
        IEnumerable<MarkCode> codedIntersection = leftCodedPoints.Intersect(rightCodedPoints);

        IOrderedEnumerable<MarkPoint<UndistortedPictureCsPoint>> orderedLeft = left.Where(point => codedIntersection.Contains(point.MarkCode)).OrderBy(point => point.MarkCode.Code);
        IOrderedEnumerable<MarkPoint<UndistortedPictureCsPoint>> orderedRight = right.Where(point => codedIntersection.Contains(point.MarkCode)).OrderBy(point => point.MarkCode.Code);
        IEnumerable<(MarkPoint<UndistortedPictureCsPoint> left, MarkPoint<UndistortedPictureCsPoint> right)> pairsByCode = orderedLeft.Zip(orderedRight);

        foreach ((MarkPoint<UndistortedPictureCsPoint> left, MarkPoint<UndistortedPictureCsPoint> right) pointPair in pairsByCode)
        {
            UndistortedPictureCsPoint leftPoint = pointPair.left.Point;
            UndistortedPictureCsPoint rightPoint = pointPair.right.Point;
            result.Add(new MarkPointPair<HomogeneousPictureCsPoint>(pointPair.left.MarkCode, leftPoint.ToHomogeneousCoordinates(),
                                                                  rightPoint.ToHomogeneousCoordinates()));
        }

        // For epipolar analyzing adding only uncoded points
        foreach (MarkPoint<UndistortedPictureCsPoint> point in left.Where(point => !point.HasCode))
        {
            leftHomo.Add(point.Point.ToHomogeneousCoordinates());
        }

        foreach (MarkPoint<UndistortedPictureCsPoint> point in right.Where(point => !point.HasCode))
        {
            rightHomo.Add(point.Point.ToHomogeneousCoordinates());
        }

        Dictionary<int, List<int>> pairsForLeft = new();
        for (int i = 0; i < leftHomo.Count; ++i)
        {
            List<int> tmp = FindPairsIndexes(leftHomo[i], rightHomo.ToList(), true, scaleBar).Indexes;
            if (tmp.Count > 0)
            {
                pairsForLeft.Add(i, tmp);
            }
        }

        Dictionary<int, List<int>> pairsForRight = new();
        for (int i = 0; i < rightHomo.Count; ++i)
        {
            List<int> tmp = FindPairsIndexes(rightHomo[i], leftHomo.ToList(), false, scaleBar).Indexes;
            if (tmp.Count > 0)
            {
                pairsForRight.Add(i, tmp);
            }
        }

        // TODO: Review and rewrite pairing method. Ambiguities between points should be resolved 
        foreach (KeyValuePair<int, List<int>> pair in pairsForLeft)
        {
            List<int> foundRight = pair.Value;
            foreach (int index in foundRight)
            {
                if (pairsForRight.TryGetValue(index, out List<int>? foundLeft))
                {
                    int indexRight = index;
                    if (foundLeft != null && foundLeft.Contains(pair.Key))
                    {
                        result.Add(new(new MarkCode(0, MarkCodeType.Uncoded), leftHomo[pair.Key], rightHomo[indexRight]));
                        right.ElementAt(indexRight).MarkCode.Code = pair.Key;
                        left.ElementAt(pair.Key).MarkCode.Code = indexRight;
                        pairsForRight.Remove(indexRight);
                        break;
                    }
                }
            }
        }

        return result;
    }
}
