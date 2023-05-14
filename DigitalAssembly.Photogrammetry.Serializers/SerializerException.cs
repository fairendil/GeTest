namespace DigitalAssembly.Photogrammetry.Serializers;

public sealed class SerializerException : Exception
{
    public SerializerException(string? message) : base(message) { }
}
