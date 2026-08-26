using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
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
        private static readonly SensitivityAnalysisService Sensitivity = new();
        private static readonly DualityService Duality = new();

        private static void Main(string[] args)
        {
            while (true)
            {
                PrintMainMenu();
                string choice = (Console.ReadLine() ?? string.Empty).Trim();

                switch (choice)
                {
                    case "1": LoadFile(); break;
                    case "2": SelectAlgorithm(); break;
                    case "3": SolveModel(); break;
                    case "4": ShowSensitivityMenu(); break;
                    case "5": ExportResults(); break;
                    case "6": return;
                    default: ShowError("Invalid menu selection. Please enter a number from 1 to 6."); break;
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
            Console.WriteLine($"Last status  : {(_lastResult == null ? "Not solved" : _lastResult.Status.ToString())}");
            Console.WriteLine("------------------------------------------------------------");
            Console.WriteLine("1. Load Programming Model");
            Console.WriteLine("2. Select Algorithm");
            Console.WriteLine("3. Solve Model");
            Console.WriteLine("4. Sensitivity Analysis & Duality");
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

                Console.ForegroundColor = ConsoleColor.Green;
                Console.WriteLine($"\n[SUCCESS] Loaded {problem.NumVariables} decision variables and {problem.NumConstraints} original constraints.");
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
                ShowError($"{GetAlgorithmName(_selectedAlgorithm)} must be supplied by the group member responsible for that algorithm.");
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
            if (_loadedProblem == null || _lastResult == null ||
                _lastResult.Status != PrimalSimplexSolver.SolutionStatus.Optimal)
            {
                ShowError("Sensitivity analysis requires a loaded model with an optimal Primal Simplex solution.");
                return;
            }

            while (true)
            {
                Console.Clear();
                Console.WriteLine("============================================================");
                Console.WriteLine("             SENSITIVITY ANALYSIS & DUALITY");
                Console.WriteLine("============================================================");
                Console.WriteLine(" 1. Display range of selected Non-Basic Variable");
                Console.WriteLine(" 2. Apply change to selected Non-Basic Variable");
                Console.WriteLine(" 3. Display range of selected Basic Variable");
                Console.WriteLine(" 4. Apply change to selected Basic Variable");
                Console.WriteLine(" 5. Display range of selected constraint RHS");
                Console.WriteLine(" 6. Apply change to selected constraint RHS");
                Console.WriteLine(" 7. Display range of coefficient in Non-Basic Variable column");
                Console.WriteLine(" 8. Apply change to coefficient in Non-Basic Variable column");
                Console.WriteLine(" 9. Add a new activity to the optimal solution");
                Console.WriteLine("10. Add a new constraint to the optimal solution");
                Console.WriteLine("11. Display shadow prices");
                Console.WriteLine("12. Apply Duality / display the Dual Programming Model");
                Console.WriteLine("13. Solve the Dual Programming Model");
                Console.WriteLine("14. Verify Strong / Weak Duality");
                Console.WriteLine(" 0. Return to main menu");
                Console.WriteLine("============================================================");
                Console.Write("Select an option (0-14): ");

                string choice = (Console.ReadLine() ?? string.Empty).Trim();
                if (choice == "0") return;

                try
                {
                    switch (choice)
                    {
                        case "1": DisplayObjectiveRange(mustBeBasic: false); break;
                        case "2": ApplyObjectiveChange(mustBeBasic: false); break;
                        case "3": DisplayObjectiveRange(mustBeBasic: true); break;
                        case "4": ApplyObjectiveChange(mustBeBasic: true); break;
                        case "5": DisplayRhsRange(); break;
                        case "6": ApplyRhsChange(); break;
                        case "7": DisplayNonBasicColumnRange(); break;
                        case "8": ApplyNonBasicColumnChange(); break;
                        case "9": AddNewActivity(); break;
                        case "10": AddNewConstraint(); break;
                        case "11": DisplayShadowPrices(); break;
                        case "12": DisplayDualModel(); break;
                        case "13": SolveDualModel(); break;
                        case "14": VerifyDuality(); break;
                        default: ShowError("Invalid sensitivity selection."); break;
                    }
                }
                catch (Exception ex) when (ex is ArgumentException || ex is InvalidOperationException || ex is FormatException)
                {
                    ShowError(ex.Message);
                }

                if (_lastResult == null || _lastResult.Status != PrimalSimplexSolver.SolutionStatus.Optimal)
                    return;
            }
        }

        private static void DisplayObjectiveRange(bool mustBeBasic)
        {
            int variable = ReadVariableNumber();
            SensitivityAnalysisService.RangeResult range = mustBeBasic
                ? Sensitivity.GetBasicVariableObjectiveRange(_lastResult!, variable)
                : Sensitivity.GetNonBasicVariableObjectiveRange(_lastResult!, variable);
            Console.Clear();
            Console.WriteLine("=== OBJECTIVE COEFFICIENT RANGE ===");
            Console.WriteLine(range);
            Pause();
        }

        private static void ApplyObjectiveChange(bool mustBeBasic)
        {
            int variable = ReadVariableNumber();
            // Validate the requested basic/non-basic classification before applying.
            _ = mustBeBasic
                ? Sensitivity.GetBasicVariableObjectiveRange(_lastResult!, variable)
                : Sensitivity.GetNonBasicVariableObjectiveRange(_lastResult!, variable);

            double newValue = ReadDouble("Enter new objective coefficient: ");
            ApplyChange(Sensitivity.ApplyObjectiveCoefficientChange(_lastResult!, variable, newValue));
        }

        private static void DisplayRhsRange()
        {
            int constraint = ReadConstraintNumber();
            var range = Sensitivity.GetRhsRange(_lastResult!, constraint);
            Console.Clear();
            Console.WriteLine("=== RHS RANGE ===");
            Console.WriteLine(range);
            Pause();
        }

        private static void ApplyRhsChange()
        {
            int constraint = ReadConstraintNumber();
            double newValue = ReadDouble("Enter new RHS value: ");
            ApplyChange(Sensitivity.ApplyRhsChange(_lastResult!, constraint, newValue));
        }

        private static void DisplayNonBasicColumnRange()
        {
            int variable = ReadVariableNumber();
            int constraint = ReadConstraintNumber();
            var range = Sensitivity.GetNonBasicColumnCoefficientRange(_lastResult!, constraint, variable);
            Console.Clear();
            Console.WriteLine("=== NON-BASIC COLUMN COEFFICIENT RANGE ===");
            Console.WriteLine(range);
            Pause();
        }

        private static void ApplyNonBasicColumnChange()
        {
            int variable = ReadVariableNumber();
            int constraint = ReadConstraintNumber();
            double newValue = ReadDouble("Enter new technological coefficient: ");
            ApplyChange(Sensitivity.ApplyNonBasicColumnCoefficientChange(
                _lastResult!, constraint, variable, newValue));
        }

        private static void AddNewActivity()
        {
            double objective = ReadDouble("Objective coefficient of the new activity: ");
            var coefficients = new List<double>();
            for (int i = 0; i < _loadedProblem!.NumConstraints; i++)
                coefficients.Add(ReadDouble($"Coefficient in constraint {i + 1}: "));
            string restriction = ReadSignRestriction();

            ApplyChange(Sensitivity.AddNewActivity(_lastResult!, objective, coefficients, restriction));
        }

        private static void AddNewConstraint()
        {
            var coefficients = new List<double>();
            for (int j = 0; j < _loadedProblem!.NumVariables; j++)
                coefficients.Add(ReadDouble($"Coefficient of x{j + 1}: "));
            string relation = ReadRelation();
            double rhs = ReadDouble("RHS value: ");

            ApplyChange(Sensitivity.AddNewConstraint(_lastResult!, coefficients, relation, rhs));
        }

        private static void DisplayShadowPrices()
        {
            var prices = Sensitivity.GetShadowPrices(_lastResult!);
            Console.Clear();
            Console.WriteLine("=== SHADOW PRICES ===");
            foreach (var price in prices)
                Console.WriteLine($"Constraint {price.ConstraintNumber}: {F3(price.ShadowPrice)}");
            Console.WriteLine("\nShadow prices are marginal objective changes per one-unit RHS increase while the current basis remains valid.");
            Pause();
        }

        private static void DisplayDualModel()
        {
            DualityService.DualBuildResult build = Duality.BuildDual(_loadedProblem!);
            Console.Clear();
            Console.WriteLine(build.Description);
            Pause();
        }

        private static void SolveDualModel()
        {
            DualityService.DualSolveResult solved = Duality.SolveDual(_loadedProblem!);
            Console.Clear();
            Console.WriteLine(solved.Build.Description);
            Console.WriteLine(solved.Result.ExecutionLog);
            Pause();
        }

        private static void VerifyDuality()
        {
            DualityService.VerificationResult verification = Duality.VerifyDuality(_loadedProblem!);
            Console.Clear();
            Console.WriteLine("=== DUALITY VERIFICATION ===");
            Console.WriteLine($"Primal status    : {verification.PrimalResult.Status}");
            Console.WriteLine($"Dual status      : {verification.DualResult.Status}");
            if (verification.PrimalResult.Status == PrimalSimplexSolver.SolutionStatus.Optimal)
                Console.WriteLine($"Primal objective : {F3(verification.PrimalResult.ObjectiveValue)}");
            if (verification.DualResult.Status == PrimalSimplexSolver.SolutionStatus.Optimal)
                Console.WriteLine($"Dual objective   : {F3(verification.DualResult.ObjectiveValue)}");
            Console.WriteLine($"Weak duality     : {(verification.WeakDualitySatisfied ? "Satisfied" : "Not verified")}");
            Console.WriteLine($"Strong duality   : {(verification.StrongDualitySatisfied ? "Satisfied" : "Not verified")}");
            Console.WriteLine();
            Console.WriteLine(verification.Message);
            Pause();
        }

        private static void ApplyChange(SensitivityAnalysisService.ChangeResult change)
        {
            _loadedProblem = change.ModifiedProblem;
            _canonicalForm = change.SolverResult.CanonicalProblem ?? CanonicalConverter.ToCanonicalForm(change.ModifiedProblem);
            _lastResult = change.SolverResult;

            Console.Clear();
            Console.WriteLine("=== APPLIED CHANGE ===");
            Console.WriteLine(change.Description);
            Console.WriteLine();
            Console.WriteLine(change.SolverResult.ExecutionLog);
            Pause();
        }

        private static int ReadVariableNumber()
        {
            int count = _loadedProblem!.NumVariables;
            Console.Write($"Select decision variable x1-x{count} (enter number 1-{count}): ");
            if (!int.TryParse(Console.ReadLine(), out int number) || number < 1 || number > count)
                throw new ArgumentException("Invalid decision-variable number.");
            return number - 1;
        }

        private static int ReadConstraintNumber()
        {
            int count = _loadedProblem!.NumConstraints;
            Console.Write($"Select original constraint (1-{count}): ");
            if (!int.TryParse(Console.ReadLine(), out int number) || number < 1 || number > count)
                throw new ArgumentException("Invalid constraint number.");
            return number - 1;
        }

        private static double ReadDouble(string prompt)
        {
            Console.Write(prompt);
            string raw = (Console.ReadLine() ?? string.Empty).Trim();
            if (!double.TryParse(raw, NumberStyles.Float, CultureInfo.InvariantCulture, out double value))
                throw new ArgumentException($"'{raw}' is not a valid number. Use a decimal point if required.");
            return value;
        }

        private static string ReadRelation()
        {
            Console.Write("Constraint relation (<=, >=, =): ");
            string relation = (Console.ReadLine() ?? string.Empty).Trim();
            if (relation is not ("<=" or ">=" or "="))
                throw new ArgumentException("Relation must be <=, >= or =.");
            return relation;
        }

        private static string ReadSignRestriction()
        {
            Console.Write("Sign restriction (+, -, urs, int, bin): ");
            string restriction = (Console.ReadLine() ?? string.Empty).Trim().ToLowerInvariant();
            if (restriction is not ("+" or "-" or "urs" or "int" or "bin"))
                throw new ArgumentException("Sign restriction must be +, -, urs, int or bin.");
            return restriction;
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

        private static string F3(double value) => value.ToString("F3", CultureInfo.InvariantCulture);
    }
}
