using System.Text.RegularExpressions;

namespace DigitalAssembly.Photogrammetry.Serializers;

public static class DoubleCsvSerializer
{
    public static double[][] LoadFromFile(string filename, int doubleCount, string delimeterRegex)
    {
        string[] lines = File.ReadAllLines(filename);
        return Serialize(doubleCount, delimeterRegex, lines).ToArray();
    }

    public static List<double[]> Serialize(int doubleCount, string delimeterRegex, string[] lines)
    {
        List<double[]> result = new();
        for (int i = 0; i < lines.Length; ++i)
        {
            string line = lines[i].Trim();
            string[] doubleStrings;
            try
            {
                doubleStrings = Regex.Split(line, delimeterRegex);
            }
            catch (Exception e)
            {
                throw new SerializerException($"Cannot split line '{i}': {e.Message}");
            }

            if (doubleStrings.Length != doubleCount)
            {
                throw new SerializerException($"Number of doubles in line '{i}' does not match 'doubleCount = {doubleCount}'");
            }
            List<double> values = new();
            foreach (string doubleString in doubleStrings)
            {
                if (!double.TryParse(doubleString, out double value))
                {
                    throw new SerializerException($"Value '{doubleString}' is not a double in line '{i}'");
                }

                values.Add(value);
            }

            result.Add(values.ToArray());
        }
        return result;
    }
}
