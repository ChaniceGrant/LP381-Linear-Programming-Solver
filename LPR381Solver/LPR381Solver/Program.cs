using System;
using System.IO;
using LPR381Solver.Models;
using LPR381Solver.Services;

namespace LPR381Solver
{
    class Program
    {
        private static LpProblem _loadedProblem = null;
        private static CanonicalProblem _canonicalForm = null;

        static void Main(string[] args)
        {
            while (true)
            {
                Console.Clear();
                Console.WriteLine("=================================================");
                Console.WriteLine("       LPR381 LINEAR PROGRAMMING SOLVER          ");
                Console.WriteLine("=================================================");
                Console.WriteLine("1. Load Input Mathematical Model File");
                Console.WriteLine("2. Solve with Primal Simplex Algorithm");
                Console.WriteLine("3. Exit");
                Console.WriteLine("=================================================");
                Console.Write("Select an option (1-3): ");

                string choice = Console.ReadLine()?.Trim();
                switch (choice)
                {
                    case "1":
                        LoadFile();
                        break;
                    case "2":
                        SolvePrimalSimplex();
                        break;
                    case "3":
                        return;
                    default:
                        ShowError("Invalid menu selection.");
                        break;
                }
            }
        }

        private static void LoadFile()
        {
            Console.Write("\nEnter input file path (e.g. input.txt): ");
            string path = Console.ReadLine()?.Trim();
            if (!File.Exists(path))
            {
                ShowError("File does not exist!");
                return;
            }

            try
            {
                _loadedProblem = InputParser.ParseFile(path);
                _canonicalForm = CanonicalConverter.ToCanonicalForm(_loadedProblem);
                Console.WriteLine($"\n[SUCCESS] Loaded model with {_loadedProblem.NumVariables} decision variables and {_loadedProblem.NumConstraints} constraints.");
                Console.WriteLine("Press any key to continue...");
                Console.ReadKey();
            }
            catch (Exception ex)
            {
                ShowError($"Failed to parse file: {ex.Message}");
            }
        }

        private static void SolvePrimalSimplex()
        {
            if (_loadedProblem == null || _canonicalForm == null)
            {
                ShowError("No problem loaded! Please load an input file first.");
                return;
            }

            var solver = new PrimalSimplexSolver();
            var result = solver.Solve(_canonicalForm);

            Console.Clear();
            Console.WriteLine(result.ExecutionLog);

            Console.Write("\nEnter output file path to save results (or press ENTER to skip): ");
            string outPath = Console.ReadLine()?.Trim();
            if (!string.IsNullOrEmpty(outPath))
            {
                try
                {
                    File.WriteAllText(outPath, result.ExecutionLog);
                    Console.WriteLine($"\n[SUCCESS] Results written to {outPath}");
                }
                catch (Exception ex)
                {
                    ShowError($"Failed to write output file: {ex.Message}");
                }
            }

            Console.WriteLine("\nPress any key to return to menu...");
            Console.ReadKey();
        }

        private static void ShowError(string msg)
        {
            Console.ForegroundColor = ConsoleColor.Red;
            Console.WriteLine($"\n[ERROR] {msg}");
            Console.ResetColor();
            Console.WriteLine("Press any key to continue...");
            Console.ReadKey();
        }
    }
}