using System;
using System.IO;
using LPR381Solver.IO;
using LPR381Solver.Models;
using LPR381Solver.Services;

namespace LPR381Solver
{
    internal static class Program
    {
        private enum AlgorithmChoice
        {
            PrimalSimplex = 1,
            RevisedPrimalSimplex = 2,
            BranchAndBoundSimplex = 3,
            CuttingPlane = 4,
            BranchAndBoundKnapsack = 5
        }

        private static LpProblem? _loadedProblem;
        private static CanonicalProblem? _canonicalForm;
        private static PrimalSimplexSolver.Result? _lastResult;
        private static AlgorithmChoice _selectedAlgorithm = AlgorithmChoice.PrimalSimplex;

        private static void Main(string[] args)
        {
            while (true)
            {
                PrintMainMenu();
                string choice = (Console.ReadLine() ?? string.Empty).Trim();

                switch (choice)
                {
                    case "1":
                        LoadFile();
                        break;
                    case "2":
                        SelectAlgorithm();
                        break;
                    case "3":
                        SolveModel();
                        break;
                    case "4":
                        ShowSensitivityMenu();
                        break;
                    case "5":
                        ExportResults();
                        break;
                    case "6":
                        return;
                    default:
                        ShowError("Invalid menu selection. Please enter a number from 1 to 6.");
                        break;
                }
            }
        }

        private static void PrintMainMenu()
        {
            Console.Clear();
            Console.WriteLine("============================================================");
            Console.WriteLine("              LPR381 OPTIMISATION SOLVER");
            Console.WriteLine("============================================================");
            Console.WriteLine($"Loaded model : {(_loadedProblem == null ? "None" : $"{_loadedProblem.NumVariables} vars, {_loadedProblem.NumConstraints} constraints")}");
            Console.WriteLine($"Algorithm    : {GetAlgorithmName(_selectedAlgorithm)}");
            Console.WriteLine("------------------------------------------------------------");
            Console.WriteLine("1. Load Programming Model");
            Console.WriteLine("2. Select Algorithm");
            Console.WriteLine("3. Solve Model");
            Console.WriteLine("4. Sensitivity Analysis");
            Console.WriteLine("5. Export Results");
            Console.WriteLine("6. Exit");
            Console.WriteLine("============================================================");
            Console.Write("Select an option (1-6): ");
        }

        private static void LoadFile()
        {
            Console.Clear();
            Console.WriteLine("=== LOAD PROGRAMMING MODEL ===");
            Console.Write("Enter input file path: ");
            string path = (Console.ReadLine() ?? string.Empty).Trim().Trim('"');

            if (string.IsNullOrWhiteSpace(path))
            {
                ShowError("No file path was entered.");
                return;
            }

            if (!File.Exists(path))
            {
                ShowError($"File does not exist: {path}");
                return;
            }

            try
            {
                LpProblem problem = InputParser.ParseFile(path);
                CanonicalProblem canonical = CanonicalConverter.ToCanonicalForm(problem);

                _loadedProblem = problem;
                _canonicalForm = canonical;
                _lastResult = null;

                Console.WriteLine();
                Console.ForegroundColor = ConsoleColor.Green;
                Console.WriteLine($"[SUCCESS] Loaded {problem.NumVariables} decision variables and {problem.NumConstraints} original constraints.");
                Console.ResetColor();
                Pause();
            }
            catch (Exception ex) when (ex is IOException || ex is FormatException || ex is ArgumentException || ex is InvalidOperationException)
            {
                ShowError($"Could not load model: {ex.Message}");
            }
        }

        private static void SelectAlgorithm()
        {
            Console.Clear();
            Console.WriteLine("=== SELECT ALGORITHM ===");
            Console.WriteLine("1. Primal Simplex");
            Console.WriteLine("2. Revised Primal Simplex");
            Console.WriteLine("3. Branch & Bound Simplex");
            Console.WriteLine("4. Cutting Plane");
            Console.WriteLine("5. Branch & Bound Knapsack");
            Console.Write("Selection (1-5): ");

            if (!int.TryParse(Console.ReadLine(), out int value) || value < 1 || value > 5)
            {
                ShowError("Invalid algorithm selection.");
                return;
            }

            _selectedAlgorithm = (AlgorithmChoice)value;
            Console.WriteLine($"\nSelected: {GetAlgorithmName(_selectedAlgorithm)}");
            Pause();
        }

        private static void SolveModel()
        {
            if (_loadedProblem == null || _canonicalForm == null)
            {
                ShowError("No model is loaded. Load an input file first.");
                return;
            }

            if (_selectedAlgorithm != AlgorithmChoice.PrimalSimplex)
            {
                ShowError(
                    $"{GetAlgorithmName(_selectedAlgorithm)} is an integration point for the group member responsible for that algorithm. " +
                    "Select Primal Simplex to run Person A's solver in this branch.");
                return;
            }

            try
            {
                var solver = new PrimalSimplexSolver();
                _lastResult = solver.Solve(_canonicalForm);

                Console.Clear();
                Console.WriteLine(_lastResult.ExecutionLog);
                Pause();
            }
            catch (Exception ex)
            {
                ShowError($"Solver failed: {ex.Message}");
            }
        }

        private static void ExportResults()
        {
            if (_lastResult == null)
            {
                ShowError("There are no solved results to export. Solve a model first.");
                return;
            }

            Console.Clear();
            Console.WriteLine("=== EXPORT RESULTS ===");
            Console.Write("Enter output text file path (e.g. output.txt): ");
            string path = (Console.ReadLine() ?? string.Empty).Trim().Trim('"');

            if (string.IsNullOrWhiteSpace(path))
            {
                ShowError("No output path was entered.");
                return;
            }

            try
            {
                OutputWriter.WriteResult(path, _lastResult);
                Console.ForegroundColor = ConsoleColor.Green;
                Console.WriteLine($"\n[SUCCESS] Results exported to: {Path.GetFullPath(path)}");
                Console.ResetColor();
                Pause();
            }
            catch (Exception ex) when (ex is IOException || ex is UnauthorizedAccessException || ex is ArgumentException)
            {
                ShowError($"Could not write output file: {ex.Message}");
            }
        }

        private static void ShowSensitivityMenu()
        {
            Console.Clear();
            Console.WriteLine("=== SENSITIVITY ANALYSIS ===");
            Console.WriteLine(" 1. Display range of selected Non-Basic Variable");
            Console.WriteLine(" 2. Apply change to selected Non-Basic Variable");
            Console.WriteLine(" 3. Display range of selected Basic Variable");
            Console.WriteLine(" 4. Apply change to selected Basic Variable");
            Console.WriteLine(" 5. Display range of constraint RHS");
            Console.WriteLine(" 6. Apply change to constraint RHS");
            Console.WriteLine(" 7. Display range of variable in Non-Basic column");
            Console.WriteLine(" 8. Apply change to variable in Non-Basic column");
            Console.WriteLine(" 9. Add new activity");
            Console.WriteLine("10. Add new constraint");
            Console.WriteLine("11. Display shadow prices");
            Console.WriteLine("12. Duality operations");
            Console.WriteLine();
            Console.WriteLine("Sensitivity calculations are provided by the group member responsible for that section.");
            Console.WriteLine("This menu is the shell/integration point required by Person A.");
            Pause();
        }

        private static string GetAlgorithmName(AlgorithmChoice choice) => choice switch
        {
            AlgorithmChoice.PrimalSimplex => "Primal Simplex",
            AlgorithmChoice.RevisedPrimalSimplex => "Revised Primal Simplex",
            AlgorithmChoice.BranchAndBoundSimplex => "Branch & Bound Simplex",
            AlgorithmChoice.CuttingPlane => "Cutting Plane",
            AlgorithmChoice.BranchAndBoundKnapsack => "Branch & Bound Knapsack",
            _ => "Unknown"
        };

        private static void ShowError(string message)
        {
            Console.ForegroundColor = ConsoleColor.Red;
            Console.WriteLine($"\n[ERROR] {message}");
            Console.ResetColor();
            Pause();
        }

        private static void Pause()
        {
            Console.WriteLine("\nPress any key to continue...");
            Console.ReadKey(true);
        }
    }
}
