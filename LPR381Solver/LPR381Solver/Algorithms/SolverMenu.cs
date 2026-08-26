using LPR381Solver.Models;
using LPR381Solver.Parsing;
using LPR381Solver.Services;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;

namespace LPR381Solver.Algorithms
{
    /// <summary>
    /// A standalone menu for the Branch and Bound Knapsack and Cutting Plane
    /// algorithms. It reads a model with the shared InputParser, lets the user
    /// choose an algorithm, prints the full working and writes it to an output
    /// text file.
    ///
    /// This exists so that these two algorithms can be developed and demonstrated
    /// independently. Once the shared UI/Menu.cs is complete it should call the
    /// same solvers directly, and this class becomes a fallback.
    /// </summary>
    public static class SolverMenu
    {
        /// <summary>Entry point for the menu loop.</summary>
        public static void Run()
        {
            LpProblem? problem = null;
            string modelSource = "(no model loaded)";

            while (true)
            {
                Console.WriteLine();
                Console.WriteLine("==========================================================");
                Console.WriteLine("   LPR381 SOLVER - KNAPSACK AND CUTTING PLANE");
                Console.WriteLine("==========================================================");
                Console.WriteLine("  Current model: " + modelSource);
                Console.WriteLine();
                Console.WriteLine("  1. Load a model from an input text file");
                Console.WriteLine("  2. Solve with the Branch and Bound Knapsack Algorithm");
                Console.WriteLine("  3. Solve with the Cutting Plane Algorithm");
                Console.WriteLine("  4. Display the loaded model");
                Console.WriteLine("  5. Run the built in demonstration cases");
                Console.WriteLine("  0. Exit");
                Console.WriteLine();
                Console.Write("  Select an option: ");

                string choice = Console.ReadLine() ?? string.Empty;
                Console.WriteLine();

                switch (choice.Trim())
                {
                    case "1":
                        LoadModel(ref problem, ref modelSource);
                        break;

                    case "2":
                        SolveKnapsack(problem);
                        break;

                    case "3":
                        SolveCuttingPlane(problem);
                        break;

                    case "4":
                        DisplayModel(problem);
                        break;

                    case "5":
                        IntegerAlgorithmDemos.RunAll();
                        break;

                    case "0":
                        Console.WriteLine("  Goodbye.");
                        return;

                    default:
                        Console.WriteLine("  '" + choice + "' is not a valid option. Please choose 0 to 5.");
                        break;
                }
            }
        }

        /// <summary>
        /// Prompts for a file path and parses the model with the shared InputParser,
        /// so that this menu accepts exactly the same file format as the rest of
        /// the program.
        /// </summary>
        private static void LoadModel(ref LpProblem? problem, ref string modelSource)
        {
            Console.Write("  Enter the input file path: ");
            string path = (Console.ReadLine() ?? string.Empty).Trim().Trim('"');

            if (string.IsNullOrWhiteSpace(path))
            {
                Console.WriteLine("  No path was entered.");
                return;
            }

            try
            {
                problem = InputParser.ParseFile(path);
                modelSource = Path.GetFileName(path);
                Console.WriteLine("  Model loaded successfully from " + modelSource + ".");
                Console.WriteLine();
                DisplayModel(problem);
            }
            catch (Exception ex)
            {
                problem = null;
                modelSource = "(no model loaded)";
                Console.WriteLine("  The file could not be read as a programming model.");
                Console.WriteLine("  Reason: " + ex.Message);
            }
        }

        /// <summary>Prints the loaded model in a readable form.</summary>
        private static void DisplayModel(LpProblem? problem)
        {
            if (problem == null)
            {
                Console.WriteLine("  No model is loaded. Use option 1 first.");
                return;
            }

            Console.WriteLine("  LOADED MODEL");
            Console.WriteLine("  ----------------------------------------------------");
            Console.Write(problem.IsMaximization ? "  max z =" : "  min z =");

            for (int j = 0; j < problem.NumVariables; j++)
            {
                Console.Write(string.Format(CultureInfo.InvariantCulture,
                    " {0}{1:F3}x{2}", problem.ObjectiveCoeffs[j] >= 0 ? "+" : "-",
                    Math.Abs(problem.ObjectiveCoeffs[j]), j + 1));
            }
            Console.WriteLine();

            for (int i = 0; i < problem.NumConstraints; i++)
            {
                Console.Write("  ");
                List<double> coefficients = problem.ConstraintCoeffs[i];

                for (int j = 0; j < coefficients.Count; j++)
                {
                    Console.Write(string.Format(CultureInfo.InvariantCulture,
                        "{0}{1:F3}x{2} ", coefficients[j] >= 0 ? "+" : "-",
                        Math.Abs(coefficients[j]), j + 1));
                }

                Console.WriteLine(string.Format(CultureInfo.InvariantCulture,
                    "{0} {1:F3}", problem.Relations[i], problem.Rhs[i]));
            }

            Console.Write("  Sign restrictions:");
            foreach (string restriction in problem.SignRestrictions)
                Console.Write(" " + restriction);
            Console.WriteLine();
        }

        /// <summary>Runs the Branch and Bound Knapsack algorithm on the loaded model.</summary>
        private static void SolveKnapsack(LpProblem? problem)
        {
            if (problem == null)
            {
                Console.WriteLine("  No model is loaded. Use option 1 first.");
                return;
            }

            try
            {
                KnapsackBranchAndBound solver = new KnapsackBranchAndBound();
                solver.Solve(problem);
                Console.WriteLine(solver.Log);
                WriteOutputFile(solver.Log, "knapsack");
            }
            catch (InvalidOperationException ex)
            {
                Console.WriteLine("  This model cannot be solved with Branch and Bound Knapsack.");
                Console.WriteLine("  Reason: " + ex.Message);
            }
        }

        /// <summary>Runs the Cutting Plane algorithm on the loaded model.</summary>
        private static void SolveCuttingPlane(LpProblem? problem)
        {
            if (problem == null)
            {
                Console.WriteLine("  No model is loaded. Use option 1 first.");
                return;
            }

            try
            {
                CuttingPlane solver = new CuttingPlane();
                solver.Solve(problem);
                Console.WriteLine(solver.Log);
                WriteOutputFile(solver.Log, "cuttingplane");
            }
            catch (InvalidOperationException ex)
            {
                Console.WriteLine("  This model cannot be solved with the Cutting Plane algorithm.");
                Console.WriteLine("  Reason: " + ex.Message);
            }
        }

        /// <summary>
        /// Writes the algorithm's full working to an output text file and tells the
        /// user where it was written. All values are already rounded to three
        /// decimals by the algorithms themselves.
        /// </summary>
        private static void WriteOutputFile(string content, string algorithmName)
        {
            try
            {
                string fileName = "output_" + algorithmName + ".txt";
                string fullPath = Path.GetFullPath(fileName);

                File.WriteAllText(fullPath, content);

                Console.WriteLine("  Results exported to: " + fullPath);
            }
            catch (Exception ex)
            {
                Console.WriteLine("  The output file could not be written.");
                Console.WriteLine("  Reason: " + ex.Message);
            }
        }
    }
}