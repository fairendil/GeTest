using DigitalAssembly.Photogrammetry.Camera.DistortionModels;
using DigitalAssembly.Photogrammetry.Geometry.CoordinateSystems;

namespace DigitalAssembly.Photogrammetry.Camera;

/// <summary>
/// Параметры внутреннего ориентирования
/// https://dassembly.atlassian.net/wiki/spaces/CVIS/pages/347078658/Camera+model
/// </summary>
public class IntrisicParameters
{
    public IntrisicParameters(double focus, PictureCsPoint principalPoint, ClassicDistortionParameters classicDistortionParameters)
    {
        PrincipalPoint = principalPoint;
        Focus = focus;
        ClassicDistortionParameters = classicDistortionParameters;
    }

    /// <summary>
    /// MatrixSize / ImageSize, which means how mm per pixel
    /// Главная точка изображения(проекция центра камеры на плоскость изображения)
    /// </summary>
    public PictureCsPoint PrincipalPoint { get; }

    /// <summary>
    /// Фокус камеры
    /// </summary>
    public double Focus { get; }

    /// <summary>
    /// In mm
    /// </summary>
    public ClassicDistortionParameters ClassicDistortionParameters { get; }

}
