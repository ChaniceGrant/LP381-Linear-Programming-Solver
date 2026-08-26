using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;

using LPR381Solver.Models;

namespace LPR381Solver.Algorithms
{
    public static class SolverMenu
    {
        public static void Run()
        {
            LPModel? model = null;
            string modelSource = "(no model loaded)";

            while (true)
            {
                Console.WriteLine();
                Console.WriteLine("==========================================================");
                Console.WriteLine("   LPR381 SOLVER");
                Console.WriteLine("==========================================================");
                Console.WriteLine("  Current model: " + modelSource);
                Console.WriteLine();
                Console.WriteLine("  1. Load a model from an input text file");
                Console.WriteLine("  2. Solve with the Branch and Bound Knapsack Algorithm");
                Console.WriteLine("  3. Solve with the Cutting Plane Algorithm");
                Console.WriteLine("  4. Display the loaded model");
                Console.WriteLine("  0. Exit");
                Console.WriteLine();
                Console.Write("  Select an option: ");

                string choice = Console.ReadLine() ?? string.Empty;
                Console.WriteLine();

                switch (choice.Trim())
                {
                    case "1":
                        LoadModel(ref model, ref modelSource);
                        break;

                    case "2":
                        SolveKnapsack(model);
                        break;

                    case "3":
                        SolveCuttingPlane(model);
                        break;

                    case "4":
                        DisplayModel(model);
                        break;

                    case "0":
                        Console.WriteLine("  Goodbye.");
                        return;

                    default:
                        Console.WriteLine("  '" + choice + "' is not a valid option. Please choose 0 to 4.");
                        break;
                }
            }
        }
        private static void LoadModel(ref LPModel? model, ref string modelSource)
        {
            Console.Write("  Enter the input file path: ");
            string path = (Console.ReadLine() ?? string.Empty).Trim().Trim('"');

            if (string.IsNullOrWhiteSpace(path))
            {
                Console.WriteLine("  No path was entered.");
                return;
            }

            if (!File.Exists(path))
            {
                Console.WriteLine("  The file '" + path + "' does not exist.");
                return;
            }

            try
            {
                model = ParseInputFile(path);
                modelSource = Path.GetFileName(path);
                Console.WriteLine("  Model loaded successfully from " + modelSource + ".");
                Console.WriteLine();
                DisplayModel(model);
            }
            catch (Exception ex)
            {
                model = null;
                modelSource = "(no model loaded)";
                Console.WriteLine("  The file could not be read as a programming model.");
                Console.WriteLine("  Reason: " + ex.Message);
            }
        }

        public static LPModel ParseInputFile(string path)
        {
            List<string> lines = new List<string>();

            foreach (string raw in File.ReadAllLines(path))
            {
                string trimmed = raw.Trim();
                if (trimmed.Length > 0)
                    lines.Add(trimmed);
            }

            if (lines.Count < 3)
                throw new InvalidOperationException(
                    "The file needs at least an objective line, one constraint line and a sign restriction line.");

            LPModel model = new LPModel();

            string[] objectiveTokens = SplitTokens(lines[0]);

            if (objectiveTokens.Length < 2)
                throw new InvalidOperationException("The objective line is incomplete.");

            string objectiveType = objectiveTokens[0].ToLowerInvariant();

            if (objectiveType != "max" && objectiveType != "min")
                throw new InvalidOperationException(
                    "The objective line must start with 'max' or 'min', but it starts with '"
                    + objectiveTokens[0] + "'.");

            model.ObjectiveType = objectiveType;

            List<double> objectiveCoefficients = new List<double>();
            for (int i = 1; i < objectiveTokens.Length; i++)
                objectiveCoefficients.Add(ParseSignedNumber(objectiveTokens[i], "objective coefficient"));

            int variableCount = objectiveCoefficients.Count;

            if (variableCount == 0)
                throw new InvalidOperationException("The objective line has no coefficients.");

            string[] signTokens = SplitTokens(lines[lines.Count - 1]);

            if (signTokens.Length != variableCount)
                throw new InvalidOperationException(
                    "There are " + variableCount + " variables but "
                    + signTokens.Length + " sign restrictions.");

            for (int j = 0; j < variableCount; j++)
            {
                string restriction = signTokens[j].ToLowerInvariant();

                if (restriction != "+" && restriction != "-" && restriction != "urs"
                    && restriction != "int" && restriction != "bin")
                {
                    throw new InvalidOperationException(
                        "'" + signTokens[j] + "' is not a valid sign restriction. "
                        + "Use +, -, urs, int or bin.");
                }

                model.Variables.Add(new Variable("x" + (j + 1), objectiveCoefficients[j], restriction));
            }

            for (int lineIndex = 1; lineIndex < lines.Count - 1; lineIndex++)
            {
                model.Constraints.Add(ParseConstraint(lines[lineIndex], variableCount, lineIndex + 1));
            }

            if (model.Constraints.Count == 0)
                throw new InvalidOperationException("The file contains no constraints.");

            return model;
        }

        private static Constraint ParseConstraint(string line, int variableCount, int lineNumber)
        {

            string spaced = line
                .Replace("<=", " <= ")
                .Replace(">=", " >= ");

            spaced = InsertSpacesAroundSingleEquals(spaced);

            string[] tokens = SplitTokens(spaced);

            int relationIndex = -1;
            for (int i = 0; i < tokens.Length; i++)
            {
                if (tokens[i] == "<=" || tokens[i] == ">=" || tokens[i] == "=")
                {
                    relationIndex = i;
                    break;
                }
            }

            if (relationIndex == -1)
                throw new InvalidOperationException(
                    "Line " + lineNumber + " has no relation. Use <=, >= or =.");

            if (relationIndex != variableCount)
                throw new InvalidOperationException(
                    "Line " + lineNumber + " has " + relationIndex + " coefficients but there are "
                    + variableCount + " variables.");

            if (relationIndex + 1 >= tokens.Length)
                throw new InvalidOperationException(
                    "Line " + lineNumber + " has no right hand side value.");

            List<double> coefficients = new List<double>();
            for (int i = 0; i < relationIndex; i++)
                coefficients.Add(ParseSignedNumber(tokens[i], "technological coefficient on line " + lineNumber));

            double rightHandSide = ParseSignedNumber(
                tokens[relationIndex + 1], "right hand side on line " + lineNumber);

            return new Constraint(coefficients, tokens[relationIndex], rightHandSide);
        }

        private static string InsertSpacesAroundSingleEquals(string text)
        {
            System.Text.StringBuilder builder = new System.Text.StringBuilder();

            for (int i = 0; i < text.Length; i++)
            {
                if (text[i] == '=' && (i == 0 || (text[i - 1] != '<' && text[i - 1] != '>' && text[i - 1] != ' ')))
                {
                    builder.Append(' ').Append('=').Append(' ');
                }
                else
                {
                    builder.Append(text[i]);
                }
            }

            return builder.ToString();
        }

        private static string[] SplitTokens(string line)
        {
            return line.Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
        }
        private static double ParseSignedNumber(string token, string description)
        {
            string cleaned = token;

            if (cleaned.StartsWith("+"))
                cleaned = cleaned.Substring(1);

            if (!double.TryParse(cleaned, NumberStyles.Float, CultureInfo.InvariantCulture, out double value))
            {
                throw new InvalidOperationException(
                    "'" + token + "' is not a valid " + description + ".");
            }

            return value;
        }

        private static void DisplayModel(LPModel? model)
        {
            if (model == null)
            {
                Console.WriteLine("  No model is loaded. Use option 1 first.");
                return;
            }

            Console.WriteLine("  LOADED MODEL");
            Console.WriteLine("  ----------------------------------------------------");
            Console.Write("  " + model.ObjectiveType + " z =");

            foreach (Variable variable in model.Variables)
            {
                Console.Write(string.Format(CultureInfo.InvariantCulture,
                    " {0}{1:F3}{2}", variable.ObjectiveCoefficient >= 0 ? "+" : "-",
                    Math.Abs(variable.ObjectiveCoefficient), variable.Name));
            }
            Console.WriteLine();

            foreach (Constraint constraint in model.Constraints)
            {
                Console.Write("  ");
                for (int j = 0; j < constraint.Coefficients.Count; j++)
                {
                    Console.Write(string.Format(CultureInfo.InvariantCulture,
                        "{0}{1:F3}{2} ", constraint.Coefficients[j] >= 0 ? "+" : "-",
                        Math.Abs(constraint.Coefficients[j]), model.Variables[j].Name));
                }
                Console.WriteLine(string.Format(CultureInfo.InvariantCulture,
                    "{0} {1:F3}", constraint.Relation, constraint.RightHandSide));
            }

            Console.Write("  Sign restrictions:");
            foreach (Variable variable in model.Variables)
                Console.Write(" " + variable.SignRestriction);
            Console.WriteLine();
        }

        private static void SolveKnapsack(LPModel? model)
        {
            if (model == null)
            {
                Console.WriteLine("  No model is loaded. Use option 1 first.");
                return;
            }

            try
            {
                KnapsackBranchAndBound solver = new KnapsackBranchAndBound();
                solver.Solve(model);
                Console.WriteLine(solver.Log);
                WriteOutputFile(solver.Log, "knapsack");
            }
            catch (InvalidOperationException ex)
            {
                Console.WriteLine("  This model cannot be solved with Branch and Bound Knapsack.");
                Console.WriteLine("  Reason: " + ex.Message);
            }
        }

        private static void SolveCuttingPlane(LPModel? model)
        {
            if (model == null)
            {
                Console.WriteLine("  No model is loaded. Use option 1 first.");
                return;
            }

            try
            {
                CuttingPlane solver = new CuttingPlane();
                solver.Solve(model);
                Console.WriteLine(solver.Log);
                WriteOutputFile(solver.Log, "cuttingplane");
            }
            catch (InvalidOperationException ex)
            {
                Console.WriteLine("  This model cannot be solved with the Cutting Plane algorithm.");
                Console.WriteLine("  Reason: " + ex.Message);
            }
        }
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