using DigitalAssembly.Photogrammetry.Geometry.CoordinateSystems;

namespace DigitalAssembly.Photogrammetry.Camera.DistortionModels;

public interface IDistortion
{
    IEnumerable<MarkPoint<UndistortedPictureCsPoint>> Undistort(IEnumerable<MarkPoint<PictureCsPoint>> points);
}
