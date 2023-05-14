using DigitalAssembly.Math.Common;

namespace DigitalAssembly.Math.PointClouds.Classification;

public interface IPcClassification
{
    /// <summary>
    /// Выделение соответствующих друг другу точек на исходном и целевом облаках точек
    /// </summary>
    /// <typeparam name="PT">Point of type Point3D</typeparam>
    /// <param name="initialPoints"></param>
    /// <param name="targetPoints"></param>
    /// <returns></returns>
    public (List<PT> initialPointsFound, List<PT> targetPointsFound) SelectPoints<PT>(List<PT> initialPoints, List<PT> targetPoints)
        where PT : Point3D<PT>;
}
