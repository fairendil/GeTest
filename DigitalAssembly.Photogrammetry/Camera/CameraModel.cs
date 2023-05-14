using DigitalAssembly.Math.Common;
using DigitalAssembly.Photogrammetry.Geometry.Common;
using DigitalAssembly.Photogrammetry.Geometry.CoordinateSystems;

namespace DigitalAssembly.Photogrammetry.Camera;

/// <summary>
/// Camera class
/// </summary>
public sealed class CameraModel
{
    public CameraModel(Transformation3D<ModelCsPoint> extrisicParameters,
                       IntrisicParameters intrisicParameters,
                       Size imageSize,
                       Size matrixSize)
    {
        ExtrisicParameters = extrisicParameters;
        IntrisicParameters = intrisicParameters;
        ImageCentre = new PixelCsPoint(imageSize.Width / 2, imageSize.Height / 2);
        ScaleParameter = GetPixelSize((decimal)matrixSize.Width, (decimal)matrixSize.Height, (int)imageSize.Width, (int)imageSize.Height);
        ImageSize = imageSize;
    }

    public CameraModel(Transformation3D<ModelCsPoint> extrisicParameters,
                       IntrisicParameters intrisicParameters,
                       PixelCsPoint imageCentre,
                       PixelSize scaleParameter,
                       Size imageSize)
    {
        ExtrisicParameters = extrisicParameters;
        IntrisicParameters = intrisicParameters;
        ImageCentre = imageCentre;
        ScaleParameter = scaleParameter;
        ImageSize = imageSize;
    }


    // TODO: Add default CameraModel

    private PixelSize GetPixelSize(decimal maxtrixWidth, decimal matrixHeight, int imageWidth, int imageHeight)
    {
        double x = (double)(matrixHeight / imageHeight);
        double y = (double)(maxtrixWidth / imageWidth);
        return new PixelSize(x, y);
    }

    /// <summary>
    /// Положение камеры относительно внешней системы координат(параметры внешней ориентации)
    /// </summary>
    public Transformation3D<ModelCsPoint> ExtrisicParameters { get; set; }

    /// <summary>
    /// Параметры внутренего ориентирования
    /// </summary>
    public IntrisicParameters IntrisicParameters { get; set; }

    /// <summary>
    /// Центр изображения для расчета точек в системе координат изображения
    /// </summary>
    public PixelCsPoint ImageCentre { get; }
    /// <summary>
    /// Pixel size in mm
    /// Размер пикселя
    /// </summary>
    public PixelSize ScaleParameter { get; }
    public Size ImageSize { get; private set; }
}
