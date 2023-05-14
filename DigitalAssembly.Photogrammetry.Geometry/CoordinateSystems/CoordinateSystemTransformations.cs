using DigitalAssembly.Math.Common;
using DigitalAssembly.Photogrammetry.Geometry.Common;
using MathNet.Numerics.LinearAlgebra;

namespace DigitalAssembly.Photogrammetry.Geometry.CoordinateSystems;

public static class CoordinateSystemTransformations
{
    public static PixelSize PixelSizeFromCameraParameters(double maxtrixWidth, double matrixHeight, double imageWidth, double imageHeight)
    {
        double x = matrixHeight / imageHeight;
        double y = maxtrixWidth / imageWidth;
        return new PixelSize(x, y);
    }

    public static PictureCsPoint ToPictureCsPoint(this PixelCsPoint point,
                                                       PixelCsPoint pictureCentre,
                                                       PixelSize pixelSize)
    {
        double U = point.X;
        double V = point.Y;
        //TODO: Добавить варианты расчета для других систем координат
        return new PictureCsPoint(pixelSize.X * (-pictureCentre.X + U), pixelSize.Y * (pictureCentre.Y - V));
    }

    public static PixelCsPoint ToPixelCSPoint(this PictureCsPoint point,
                                              PixelCsPoint pictureCentre,
                                              PixelSize pixelSize)
        => new(point.X / pixelSize.X + pictureCentre.X, -point.Y / pixelSize.Y + pictureCentre.Y);


    public static UndistortedPictureCsPoint ToUndistortedPoint(this PictureCsPoint point, PictureCsPoint distortion)
    {
        PictureCsPoint undistorted = point - distortion;
        return new UndistortedPictureCsPoint(undistorted.X, undistorted.Y);
    }

    public static HomogeneousPictureCsPoint ToHomogeneousCoordinates(this UndistortedPictureCsPoint point)
        => new(point.X, point.Y, 1);

    public static CameraCsPoint ToCameraCsPoint(this HomogeneousPictureCsPoint point, PictureCsPoint principalPoint, double cameraFocusDistance)
    {
        return System.Math.Abs(point.Z - 1) < 1e-6
            ? (new(point.X - principalPoint.X, point.Y - principalPoint.Y, cameraFocusDistance))
            : new(point.X / point.Z - principalPoint.X, point.Y / point.Z - principalPoint.Y, cameraFocusDistance);
    }

    public static CameraCsPoint ToCameraCsPoint(this PictureCsPoint point, PictureCsPoint principalPoint, double cameraFocusDistance)
        => new(point.X - principalPoint.X, point.Y - principalPoint.Y, cameraFocusDistance);

    public static PictureCsPoint ToPictureCsPoint(this CameraCsPoint point, PictureCsPoint principalPoint)
        => new(point.X + principalPoint.X, point.Y + principalPoint.Y);

    public static ModelCsPoint BuildModelCsPoint(CameraCsPoint point, Transformation3D<ModelCsPoint> cameraPosition)
    {
        Vector<double> modelPoint = cameraPosition.TransformationMatrix * Vector<double>.Build.DenseOfArray(new double[] { point.X, point.Y, point.Z, 1 });
        return new ModelCsPoint(modelPoint[0], modelPoint[1], modelPoint[2]);
    }

    public static CameraCsPoint ToCameraCsPoint(this ModelCsPoint point, Transformation3D<ModelCsPoint> cameraPosition)
    {
        Vector<double> modelPoint = cameraPosition.TransformationMatrix.TransposeThisAndMultiply(Vector<double>.Build.DenseOfArray(new double[] { point.X, point.Y, point.Z, 1 }));
        return new CameraCsPoint(modelPoint[0], modelPoint[1], modelPoint[2]);
    }
}

