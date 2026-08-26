using System;
using System.IO;
using System.Text;
using LPR381Solver.Services;

namespace LPR381Solver.IO
{
    public static class OutputWriter
    {
        public static void WriteResult(string filePath, PrimalSimplexSolver.Result result)
        {
            if (string.IsNullOrWhiteSpace(filePath))
                throw new ArgumentException("Output file path cannot be empty.", nameof(filePath));

            ArgumentNullException.ThrowIfNull(result);

            string? directory = Path.GetDirectoryName(Path.GetFullPath(filePath));
            if (!string.IsNullOrEmpty(directory) && !Directory.Exists(directory))
                Directory.CreateDirectory(directory);

            File.WriteAllText(filePath, result.ExecutionLog, new UTF8Encoding(false));
        }
    }
}
