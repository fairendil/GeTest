using DigitalAssembly.Photogrammetry.Camera;
using System.ComponentModel;

namespace DigitalAssembly.Photogrammetry.Calibration;

public class CalibrationInitialGuess
{
    CameraModel[] cameraModels;
    // default: XYZABC = 0, focus ~ real, matrix size = (m, n), image size = (M, N), x0 = 0, y0 = 0
}
