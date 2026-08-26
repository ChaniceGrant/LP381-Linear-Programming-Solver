using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using LPR381Solver.Models;

namespace LPR381Solver.Services
{
    public class PrimalSimplexSolver
    {
        public enum SolutionStatus
        {
            Optimal,
            Unbounded,
            Infeasible
        }

        public class Result
        {
            public SolutionStatus Status { get; set; }
            public double ObjectiveValue { get; set; }
            public double[] VariableValues { get; set; } = Array.Empty<double>();
            public string ExecutionLog { get; set; } = string.Empty;
        }

        private const double Epsilon = 1e-9;
        private const int MaxIterations = 1000;

        public Result Solve(CanonicalProblem canonical)
        {
            ArgumentNullException.ThrowIfNull(canonical);
            ValidateCanonical(canonical);

            var log = new StringBuilder();
            log.AppendLine("=== CANONICAL FORM ===");
            log.AppendLine(FormatCanonicalText(canonical));

            double[,] tableau = (double[,])canonical.TableauMatrix.Clone();
            int m = canonical.NumConstraints;
            int n = canonical.NumVarsTotal;
            var basis = new List<int>(canonical.BasicVariables);
            var artificial = new HashSet<int>(canonical.ArtificialVarIndices);

            if (artificial.Count > 0)
            {
                log.AppendLine("=== PHASE 1: FIND FEASIBLE BASIS ===");
                BuildPhaseOneObjective(tableau, basis, artificial, m, n);

                SolutionStatus phaseOneStatus = RunSimplexLoop(
                    tableau, basis, m, n, canonical.VariableNames, log, null, "Phase 1");

                if (phaseOneStatus == SolutionStatus.Unbounded || tableau[0, n] < -1e-7)
                {
                    log.AppendLine("[RESULT] Model is INFEASIBLE (Phase 1 objective is not zero).\n");
                    return new Result
                    {
                        Status = SolutionStatus.Infeasible,
                        ExecutionLog = log.ToString()
                    };
                }

                PivotArtificialBasicsOut(tableau, basis, artificial, m, n, canonical.VariableNames, log);

                log.AppendLine("=== PHASE 2: OPTIMIZE ORIGINAL OBJECTIVE ===");
                ReconstructOriginalObjective(tableau, canonical, basis, m, n);
            }
            else
            {
                log.AppendLine("=== PRIMAL SIMPLEX ITERATIONS ===");
            }

            SolutionStatus status = RunSimplexLoop(
                tableau,
                basis,
                m,
                n,
                canonical.VariableNames,
                log,
                artificial,
                "Phase 2");

            if (status == SolutionStatus.Unbounded)
            {
                log.AppendLine("[RESULT] Model is UNBOUNDED.");
                return new Result
                {
                    Status = SolutionStatus.Unbounded,
                    ExecutionLog = log.ToString()
                };
            }

            double[] canonicalValues = new double[n];
            for (int i = 0; i < m; i++)
            {
                int basicColumn = basis[i];
                if (basicColumn >= 0 && basicColumn < n)
                    canonicalValues[basicColumn] = tableau[i + 1, n];
            }

            double[] originalValues = ExtractOriginalValues(canonical, canonicalValues);
            double objectiveValue = tableau[0, n];
            if (!canonical.Original.IsMaximization)
                objectiveValue = -objectiveValue;

            CleanNearZero(originalValues);
            if (Math.Abs(objectiveValue) < Epsilon)
                objectiveValue = 0.0;

            log.AppendLine("[RESULT] OPTIMAL SOLUTION FOUND");
            log.AppendLine($"Objective Value (z) = {F3(objectiveValue)}");
            log.AppendLine("Variable Values:");
            for (int i = 0; i < originalValues.Length; i++)
                log.AppendLine($"  x{i + 1} = {F3(originalValues[i])}");

            return new Result
            {
                Status = SolutionStatus.Optimal,
                ObjectiveValue = objectiveValue,
                VariableValues = originalValues,
                ExecutionLog = log.ToString()
            };
        }

        private static void BuildPhaseOneObjective(
            double[,] tableau,
            List<int> basis,
            HashSet<int> artificial,
            int m,
            int n)
        {
            for (int j = 0; j <= n; j++)
                tableau[0, j] = 0.0;

            // Maximise -W. Since c_artificial = -1, z - c*x has +1 in each art column.
            foreach (int col in artificial)
                tableau[0, col] = 1.0;

            // Put Phase 1 Row 0 in canonical form with respect to the current basis.
            for (int i = 0; i < m; i++)
            {
                int basic = basis[i];
                if (!artificial.Contains(basic))
                    continue;

                double factor = tableau[0, basic];
                for (int j = 0; j <= n; j++)
                    tableau[0, j] -= factor * tableau[i + 1, j];
            }
        }

        private static void PivotArtificialBasicsOut(
            double[,] tableau,
            List<int> basis,
            HashSet<int> artificial,
            int m,
            int n,
            List<string> variableNames,
            StringBuilder log)
        {
            var basicSet = new HashSet<int>(basis);

            for (int i = 0; i < m; i++)
            {
                if (!artificial.Contains(basis[i]))
                    continue;

                int row = i + 1;
                int replacement = -1;

                for (int col = 0; col < n; col++)
                {
                    if (artificial.Contains(col) || basicSet.Contains(col))
                        continue;
                    if (Math.Abs(tableau[row, col]) > Epsilon)
                    {
                        replacement = col;
                        break;
                    }
                }

                if (replacement == -1)
                    continue; // Redundant zero row; art variable remains at value zero and is forbidden from re-entering.

                int oldBasic = basis[i];
                Pivot(tableau, row, replacement, m, n);
                basis[i] = replacement;
                basicSet.Remove(oldBasic);
                basicSet.Add(replacement);
                log.AppendLine($"Removed artificial basic variable {variableNames[oldBasic]} using {variableNames[replacement]}.");
            }
        }

        private static void ReconstructOriginalObjective(
            double[,] tableau,
            CanonicalProblem canonical,
            List<int> basis,
            int m,
            int n)
        {
            for (int j = 0; j <= n; j++)
                tableau[0, j] = canonical.TableauMatrix[0, j];

            for (int i = 0; i < m; i++)
            {
                int basic = basis[i];
                if (basic < 0 || basic >= n)
                    continue;

                double factor = tableau[0, basic];
                if (Math.Abs(factor) <= Epsilon)
                    continue;

                for (int j = 0; j <= n; j++)
                    tableau[0, j] -= factor * tableau[i + 1, j];
            }

            CleanTableau(tableau, m, n);
        }

        private static SolutionStatus RunSimplexLoop(
            double[,] tableau,
            List<int> basis,
            int m,
            int n,
            List<string> variableNames,
            StringBuilder log,
            HashSet<int>? forbiddenEnteringColumns,
            string phaseName)
        {
            for (int iteration = 0; iteration < MaxIterations; iteration++)
            {
                log.AppendLine($"--- {phaseName} Iteration {iteration} ---");
                log.AppendLine(FormatTableau(tableau, basis, m, n, variableNames));

                int pivotColumn = -1;
                double mostNegative = -Epsilon;

                for (int j = 0; j < n; j++)
                {
                    if (forbiddenEnteringColumns != null && forbiddenEnteringColumns.Contains(j))
                        continue;

                    if (tableau[0, j] < mostNegative)
                    {
                        mostNegative = tableau[0, j];
                        pivotColumn = j;
                    }
                }

                if (pivotColumn == -1)
                    return SolutionStatus.Optimal;

                int pivotRow = -1;
                double minimumRatio = double.PositiveInfinity;

                for (int i = 1; i <= m; i++)
                {
                    double coefficient = tableau[i, pivotColumn];
                    if (coefficient <= Epsilon)
                        continue;

                    double ratio = tableau[i, n] / coefficient;
                    if (ratio >= -Epsilon && ratio < minimumRatio - Epsilon)
                    {
                        minimumRatio = ratio;
                        pivotRow = i;
                    }
                }

                if (pivotRow == -1)
                {
                    log.AppendLine($"Pivot column: {pivotColumn + 1} ({variableNames[pivotColumn]})");
                    log.AppendLine("No valid positive ratio exists for the pivot column.");
                    return SolutionStatus.Unbounded;
                }

                double pivotElement = tableau[pivotRow, pivotColumn];
                string leaving = variableNames[basis[pivotRow - 1]];

                log.AppendLine($"Entering variable: {variableNames[pivotColumn]}");
                log.AppendLine($"Leaving variable: {leaving}");
                log.AppendLine($"Pivot column: {pivotColumn + 1} ({variableNames[pivotColumn]})");
                log.AppendLine($"Pivot row: {pivotRow}");
                log.AppendLine($"Minimum ratio: {F3(minimumRatio)}");
                log.AppendLine($"Pivot element: {F3(pivotElement)}");

                Pivot(tableau, pivotRow, pivotColumn, m, n);
                basis[pivotRow - 1] = pivotColumn;
            }

            throw new InvalidOperationException(
                $"Simplex stopped after {MaxIterations} iterations. The model may be cycling or numerically unstable.");
        }

        private static void Pivot(double[,] tableau, int pivotRow, int pivotColumn, int m, int n)
        {
            double pivot = tableau[pivotRow, pivotColumn];
            if (Math.Abs(pivot) <= Epsilon)
                throw new InvalidOperationException("Cannot pivot on a zero element.");

            for (int j = 0; j <= n; j++)
                tableau[pivotRow, j] /= pivot;

            for (int i = 0; i <= m; i++)
            {
                if (i == pivotRow)
                    continue;

                double factor = tableau[i, pivotColumn];
                if (Math.Abs(factor) <= Epsilon)
                    continue;

                for (int j = 0; j <= n; j++)
                    tableau[i, j] -= factor * tableau[pivotRow, j];
            }

            CleanTableau(tableau, m, n);
        }

        private static double[] ExtractOriginalValues(CanonicalProblem canonical, double[] canonicalValues)
        {
            var result = new double[canonical.Original.NumVariables];

            // Backwards-compatible fallback for older canonical objects.
            if (canonical.OriginalVariableColumns.Count != canonical.Original.NumVariables)
            {
                for (int i = 0; i < result.Length && i < canonicalValues.Length; i++)
                    result[i] = canonicalValues[i];
                return result;
            }

            for (int original = 0; original < result.Length; original++)
            {
                for (int k = 0; k < canonical.OriginalVariableColumns[original].Count; k++)
                {
                    int col = canonical.OriginalVariableColumns[original][k];
                    double multiplier = canonical.OriginalVariableMultipliers[original][k];
                    result[original] += multiplier * canonicalValues[col];
                }
            }

            return result;
        }

        private static string FormatTableau(
            double[,] tableau,
            List<int> basis,
            int m,
            int n,
            List<string> variableNames)
        {
            var sb = new StringBuilder();
            sb.Append(string.Format(CultureInfo.InvariantCulture, "{0,-12}", "Basic"));
            foreach (string variable in variableNames)
                sb.Append(string.Format(CultureInfo.InvariantCulture, "{0,11}", variable));
            sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "{0,11}", "RHS"));

            sb.Append(string.Format(CultureInfo.InvariantCulture, "{0,-12}", "z"));
            for (int j = 0; j <= n; j++)
                sb.Append(string.Format(CultureInfo.InvariantCulture, "{0,11:F3}", tableau[0, j]));
            sb.AppendLine();

            for (int i = 1; i <= m; i++)
            {
                int basicIndex = basis[i - 1];
                string basicName = basicIndex >= 0 && basicIndex < variableNames.Count
                    ? variableNames[basicIndex]
                    : "?";

                sb.Append(string.Format(CultureInfo.InvariantCulture, "{0,-12}", basicName));
                for (int j = 0; j <= n; j++)
                    sb.Append(string.Format(CultureInfo.InvariantCulture, "{0,11:F3}", tableau[i, j]));
                sb.AppendLine();
            }

            return sb.ToString();
        }

        private static string FormatCanonicalText(CanonicalProblem canonical)
        {
            var sb = new StringBuilder();
            sb.Append(canonical.Original.IsMaximization ? "max z = " : "min z = ");
            sb.AppendLine(FormatOriginalObjective(canonical.Original));
            sb.AppendLine("Subject to (canonical equalities):");

            for (int i = 0; i < canonical.NumConstraints; i++)
            {
                var terms = new List<string>();
                for (int j = 0; j < canonical.NumVarsTotal; j++)
                {
                    double value = canonical.TableauMatrix[i + 1, j];
                    if (Math.Abs(value) <= Epsilon)
                        continue;

                    string magnitude = F3(Math.Abs(value));
                    string term = $"{magnitude}*{canonical.VariableNames[j]}";
                    if (terms.Count == 0)
                        terms.Add(value < 0 ? "- " + term : term);
                    else
                        terms.Add((value < 0 ? "- " : "+ ") + term);
                }

                sb.AppendLine($"  {string.Join(" ", terms)} = {F3(canonical.TableauMatrix[i + 1, canonical.NumVarsTotal])}");
            }

            sb.AppendLine("All canonical variables are >= 0.");
            return sb.ToString();
        }

        private static string FormatOriginalObjective(LpProblem lp)
        {
            var parts = new List<string>();
            for (int i = 0; i < lp.NumVariables; i++)
            {
                double c = lp.ObjectiveCoeffs[i];
                string term = $"{F3(Math.Abs(c))}*x{i + 1}";
                if (i == 0)
                    parts.Add(c < 0 ? "- " + term : term);
                else
                    parts.Add((c < 0 ? "- " : "+ ") + term);
            }
            return string.Join(" ", parts);
        }

        private static void ValidateCanonical(CanonicalProblem canonical)
        {
            if (canonical.BasicVariables.Count != canonical.NumConstraints)
                throw new InvalidOperationException("Canonical basis size does not match the constraint count.");
            if (canonical.TableauMatrix.GetLength(0) != canonical.NumConstraints + 1 ||
                canonical.TableauMatrix.GetLength(1) != canonical.NumVarsTotal + 1)
                throw new InvalidOperationException("Canonical tableau dimensions are inconsistent.");
        }

        private static void CleanTableau(double[,] tableau, int m, int n)
        {
            for (int i = 0; i <= m; i++)
                for (int j = 0; j <= n; j++)
                    if (Math.Abs(tableau[i, j]) < Epsilon)
                        tableau[i, j] = 0.0;
        }

        private static void CleanNearZero(double[] values)
        {
            for (int i = 0; i < values.Length; i++)
                if (Math.Abs(values[i]) < Epsilon)
                    values[i] = 0.0;
        }

        private static string F3(double value) =>
            value.ToString("F3", CultureInfo.InvariantCulture);
    }
}
