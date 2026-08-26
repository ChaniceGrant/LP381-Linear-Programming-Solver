using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using LPR381Solver.Models;

namespace LPR381Solver.Services
{
    public static class InputParser
    {
        public static LpProblem ParseFile(string filePath)
        {
            if (string.IsNullOrWhiteSpace(filePath))
                throw new ArgumentException("Input file path cannot be empty.", nameof(filePath));
            if (!File.Exists(filePath))
                throw new FileNotFoundException("The specified input file could not be found.", filePath);

            string[] lines = File.ReadAllLines(filePath)
                .Where(line => !string.IsNullOrWhiteSpace(line))
                .Select(line => line.Trim())
                .ToArray();

            if (lines.Length < 3)
                throw new FormatException("Input file must contain an objective, at least one constraint, and a sign-restriction line.");

            var problem = new LpProblem();
            ParseObjective(lines[0], problem);
            int numVariables = problem.NumVariables;
            ParseRestrictions(lines[^1], problem, numVariables);

            for (int lineIndex = 1; lineIndex < lines.Length - 1; lineIndex++)
                ParseConstraint(lines[lineIndex], lineIndex + 1, problem, numVariables);

            return problem;
        }

        private static void ParseObjective(string line, LpProblem problem)
        {
            List<string> tokens = Tokenize(line);
            if (tokens.Count < 2)
                throw new FormatException("Objective line must contain max/min followed by coefficients.");

            string direction = tokens[0].ToLowerInvariant();
            problem.IsMaximization = direction switch
            {
                "max" => true,
                "min" => false,
                _ => throw new FormatException("Objective must begin with either 'max' or 'min'.")
            };

            problem.ObjectiveCoeffs = ParseCoefficients(tokens.Skip(1).ToList());
            if (problem.ObjectiveCoeffs.Count == 0)
                throw new FormatException("At least one objective coefficient is required.");
        }

        private static void ParseRestrictions(string line, LpProblem problem, int numVariables)
        {
            List<string> restrictions = Tokenize(line)
                .Select(x => x.ToLowerInvariant())
                .ToList();

            if (restrictions.Count != numVariables)
                throw new FormatException($"Expected {numVariables} sign restrictions, but found {restrictions.Count}.");

            foreach (string restriction in restrictions)
            {
                if (restriction != "+" && restriction != "-" && restriction != "urs" &&
                    restriction != "int" && restriction != "bin")
                {
                    throw new FormatException(
                        $"Invalid sign restriction '{restriction}'. Valid restrictions are +, -, urs, int and bin.");
                }
            }

            problem.SignRestrictions = restrictions;
        }

        private static void ParseConstraint(
            string line,
            int humanLineNumber,
            LpProblem problem,
            int numVariables)
        {
            List<string> tokens = Tokenize(line);
            int relationIndex = tokens.FindIndex(t => t == "<=" || t == ">=" || t == "=");

            if (relationIndex <= 0)
                throw new FormatException($"Missing or misplaced constraint relation (=, <= or >=) on line {humanLineNumber}.");

            List<double> coefficients = ParseCoefficients(tokens.Take(relationIndex).ToList());
            if (coefficients.Count != numVariables)
            {
                throw new FormatException(
                    $"Constraint line {humanLineNumber} contains {coefficients.Count} coefficients, but {numVariables} were expected.");
            }

            List<string> rhsTokens = tokens.Skip(relationIndex + 1).ToList();
            double rhs = ParseSingleSignedNumber(rhsTokens, $"RHS on line {humanLineNumber}");

            problem.ConstraintCoeffs.Add(coefficients);
            problem.Relations.Add(tokens[relationIndex]);
            problem.Rhs.Add(rhs);
        }

        private static List<string> Tokenize(string line) =>
            line.Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries).ToList();

        private static List<double> ParseCoefficients(List<string> tokens)
        {
            var result = new List<double>();

            for (int i = 0; i < tokens.Count; i++)
            {
                string token = tokens[i];

                if (token == "+" || token == "-")
                {
                    if (i + 1 >= tokens.Count || !TryParseNumber(tokens[i + 1], out double magnitude))
                        throw new FormatException($"Sign '{token}' must be followed by a numeric coefficient.");
                    if (magnitude < 0)
                        throw new FormatException("Do not provide two signs for one coefficient.");

                    result.Add(token == "-" ? -magnitude : magnitude);
                    i++;
                    continue;
                }

                if (!TryParseNumber(token, out double value))
                    throw new FormatException($"Invalid coefficient token '{token}'.");

                result.Add(value);
            }

            return result;
        }

        private static double ParseSingleSignedNumber(List<string> tokens, string description)
        {
            if (tokens.Count == 1 && TryParseNumber(tokens[0], out double direct))
                return direct;

            if (tokens.Count == 2 && (tokens[0] == "+" || tokens[0] == "-") &&
                TryParseNumber(tokens[1], out double magnitude) && magnitude >= 0)
            {
                return tokens[0] == "-" ? -magnitude : magnitude;
            }

            throw new FormatException($"{description} must contain exactly one numeric value (for example 40, -5, or - 5).");
        }

        private static bool TryParseNumber(string value, out double result) =>
            double.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out result);
    }
}
