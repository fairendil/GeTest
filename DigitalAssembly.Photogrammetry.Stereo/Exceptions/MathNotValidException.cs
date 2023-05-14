using System.Runtime.Serialization;

namespace DigitalAssembly.Photogrammetry.Stereo.Exceptions;
[Serializable]
public class MathNotValidException : Exception
{
    public MathNotValidException(string? message) : base(message)
    {
    }

    public MathNotValidException(string? message, Exception? innerException) : base(message, innerException)
    {
    }
}