using DigitalAssembly.Photogrammetry.Camera;

namespace DigitalAssembly.Photogrammetry;

public sealed class Project
{
    public Project(CameraModel leftCameraModel, CameraModel rightCameraModel, StereoGeometryParameters stereoGeometryParameters, double myu = 1)
    {
        LeftCameraModel = leftCameraModel;
        RightCameraModel = rightCameraModel;
        ProjectStereoGeometryParameters = stereoGeometryParameters;
        Myu = myu;
    }

    public static Project FromProjectAndMyu(Project project, double myu)
        => new(project.LeftCameraModel, project.RightCameraModel, project.ProjectStereoGeometryParameters, myu);

    public CameraModel LeftCameraModel { get; }
    public CameraModel RightCameraModel { get; }
    public StereoGeometryParameters ProjectStereoGeometryParameters { get; }
    public double Myu { get; }
}
