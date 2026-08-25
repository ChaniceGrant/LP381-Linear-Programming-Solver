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
            {
                throw new ArgumentException(
                    "Input file path cannot be empty.");
            }

            if (!File.Exists(filePath))
            {
                throw new FileNotFoundException(
                    "The specified input file could not be found.",
                    filePath);
            }

            string[] rawLines = File.ReadAllLines(filePath)
                .Where(line => !string.IsNullOrWhiteSpace(line))
                .Select(line => line.Trim())
                .ToArray();

            if (rawLines.Length < 2)
            {
                throw new FormatException(
                    "Input file must contain an objective line " +
                    "and a sign-restriction line.");
            }

            var problem = new LpProblem();

            // ============================================================
            // OBJECTIVE FUNCTION
            // ============================================================

            List<string> objectiveTokens = Tokenize(rawLines[0]);

            if (objectiveTokens.Count < 2)
            {
                throw new FormatException(
                    "Objective line must contain max or min " +
                    "followed by coefficients.");
            }

            string objectiveType =
                objectiveTokens[0].ToLowerInvariant();

            if (objectiveType == "max")
            {
                problem.IsMaximization = true;
            }
            else if (objectiveType == "min")
            {
                problem.IsMaximization = false;
            }
            else
            {
                throw new FormatException(
                    "Objective must begin with either 'max' or 'min'.");
            }

            List<double> objectiveCoefficients =
                ParseCoefficients(
                    objectiveTokens.Skip(1).ToList());

            if (objectiveCoefficients.Count == 0)
            {
                throw new FormatException(
                    "At least one objective coefficient is required.");
            }

            problem.ObjectiveCoeffs = objectiveCoefficients;

            int numVariables = objectiveCoefficients.Count;

            // ============================================================
            // SIGN RESTRICTIONS
            // ============================================================

            List<string> signTokens =
                Tokenize(rawLines[rawLines.Length - 1]);

            if (signTokens.Count != numVariables)
            {
                throw new FormatException(
                    $"Expected {numVariables} sign restrictions, " +
                    $"but found {signTokens.Count}.");
            }

            foreach (string token in signTokens)
            {
                string sign = token.ToLowerInvariant();

                if (sign != "+" &&
                    sign != "-" &&
                    sign != "urs" &&
                    sign != "int" &&
                    sign != "bin")
                {
                    throw new FormatException(
                        $"Invalid sign restriction '{token}'. " +
                        "Valid restrictions are +, -, urs, int and bin.");
                }

                problem.SignRestrictions.Add(sign);
            }

            // ============================================================
            // CONSTRAINTS
            // ============================================================

            for (int lineIndex = 1;
                 lineIndex < rawLines.Length - 1;
                 lineIndex++)
            {
                List<string> tokens =
                    Tokenize(rawLines[lineIndex]);

                int relationIndex = tokens.FindIndex(
                    token =>
                        token == "<=" ||
                        token == ">=" ||
                        token == "=");

                if (relationIndex == -1)
                {
                    throw new FormatException(
                        $"Missing constraint relation (=, <= or >=) " +
                        $"on line {lineIndex + 1}.");
                }

                if (relationIndex == 0)
                {
                    throw new FormatException(
                        $"Constraint on line {lineIndex + 1} " +
                        "has no coefficients.");
                }

                List<string> coefficientTokens =
                    tokens.Take(relationIndex).ToList();

                List<double> rowCoefficients =
                    ParseCoefficients(coefficientTokens);

                if (rowCoefficients.Count != numVariables)
                {
                    throw new FormatException(
                        $"Constraint line {lineIndex + 1} contains " +
                        $"{rowCoefficients.Count} coefficients, " +
                        $"but {numVariables} were expected.");
                }

                if (relationIndex + 1 >= tokens.Count)
                {
                    throw new FormatException(
                        $"Missing right-hand side value " +
                        $"on line {lineIndex + 1}.");
                }

                if (!TryParseNumber(
                        tokens[relationIndex + 1],
                        out double rhs))
                {
                    throw new FormatException(
                        $"Invalid RHS value " +
                        $"'{tokens[relationIndex + 1]}' " +
                        $"on line {lineIndex + 1}.");
                }

                if (relationIndex + 2 < tokens.Count)
                {
                    throw new FormatException(
                        $"Unexpected data after RHS " +
                        $"on line {lineIndex + 1}.");
                }

                problem.ConstraintCoeffs.Add(rowCoefficients);
                problem.Relations.Add(tokens[relationIndex]);
                problem.Rhs.Add(rhs);
            }

            if (problem.ConstraintCoeffs.Count == 0)
            {
                throw new FormatException(
                    "The programming model must contain " +
                    "at least one constraint.");
            }

            return problem;
        }

        // ================================================================
        // TOKENIZATION
        // ================================================================

        private static List<string> Tokenize(string line)
        {
            return line
                .Split(
                    new[] { ' ', '\t' },
                    StringSplitOptions.RemoveEmptyEntries)
                .ToList();
        }

        // ================================================================
        // COEFFICIENT PARSING
        // ================================================================

        private static List<double> ParseCoefficients(
            List<string> tokens)
        {
            var coefficients = new List<double>();

            for (int i = 0; i < tokens.Count; i++)
            {
                string token = tokens[i].Trim();

                // Separate sign and number.
                // Example: + 2 or - 3
                if (token == "+" || token == "-")
                {
                    if (i + 1 >= tokens.Count)
                    {
                        throw new FormatException(
                            $"Sign '{token}' is not followed by a number.");
                    }

                    if (!TryParseNumber(
                            tokens[i + 1],
                            out double value))
                    {
                        throw new FormatException(
                            $"Expected a number after '{token}', " +
                            $"but found '{tokens[i + 1]}'.");
                    }

                    if (token == "-")
                    {
                        value = -value;
                    }

                    coefficients.Add(value);

                    i++;
                    continue;
                }

                // Fused sign and number.
                // Example: +2 or -3
                if (TryParseNumber(
                        token,
                        out double directValue))
                {
                    coefficients.Add(directValue);
                    continue;
                }

                throw new FormatException(
                    $"Invalid coefficient token '{token}'.");
            }

            return coefficients;
        }

        // ================================================================
        // NUMBER PARSING
        // ================================================================

        private static bool TryParseNumber(
            string value,
            out double result)
        {
            return double.TryParse(
                value,
                NumberStyles.Float,
                CultureInfo.InvariantCulture,
                out result);
        }
    }
}