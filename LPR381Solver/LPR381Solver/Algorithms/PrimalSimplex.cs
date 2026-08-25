using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using LPR381Solver.Models;

namespace LPR381Solver.Services
{
    public class PrimalSimplexSolver
    {
        public enum SolutionStatus { Optimal, Unbounded, Infeasible }

        public class Result
        {
            public SolutionStatus Status { get; set; }
            public double ObjectiveValue { get; set; }
            public double[] VariableValues { get; set; } = Array.Empty<double>();
            public string ExecutionLog { get; set; } = string.Empty;
        }

        public Result Solve(CanonicalProblem canonical)
        {
            var log = new StringBuilder();
            log.AppendLine("=== CANONICAL FORM ===");
            log.AppendLine(FormatCanonicalText(canonical));

            double[,] T = (double[,])canonical.TableauMatrix.Clone();
            int m = canonical.NumConstraints;
            int n = canonical.NumVarsTotal;
            var basis = new List<int>(canonical.BasicVariables);

            // Phase 1 check if artificial variables exist
            if (canonical.ArtificialVarIndices.Count > 0)
            {
                log.AppendLine("\n=== PHASE 1: FIND FEASIBLE BASIS ===");
                // Construct Phase 1 Row 0: Min sum(Artificials)
                double[,] phase1T = (double[,])T.Clone();
                for (int j = 0; j <= n; j++) phase1T[0, j] = 0;

                foreach (int artIdx in canonical.ArtificialVarIndices)
                {
                    int row = basis.IndexOf(artIdx) + 1;
                    if (row > 0)
                    {
                        for (int j = 0; j <= n; j++) phase1T[0, j] -= T[row, j];
                    }
                }

                var p1Result = RunSimplexLoop(phase1T, basis, m, n, canonical.VariableNames, log);
                if (p1Result == SolutionStatus.Unbounded || Math.Abs(phase1T[0, n]) > 1e-4)
                {
                    log.AppendLine("\n[RESULT] Model is INFEASIBLE (Artificial variables cannot be driven to zero).");
                    return new Result { Status = SolutionStatus.Infeasible, ExecutionLog = log.ToString() };
                }

                // Copy solved basis matrix to main tableau & reconstruct original Row 0
                T = phase1T;
                log.AppendLine("\n=== PHASE 2: OPTIMIZE OBJECTIVE ===");
                ReconstructRowZero(T, canonical, basis, m, n);
            }
            else
            {
                log.AppendLine("\n=== PRIMAL SIMPLEX ITERATIONS ===");
            }

            var status = RunSimplexLoop(T, basis, m, n, canonical.VariableNames, log);

            if (status == SolutionStatus.Unbounded)
            {
                log.AppendLine("\n[RESULT] Model is UNBOUNDED.");
                return new Result { Status = SolutionStatus.Unbounded, ExecutionLog = log.ToString() };
            }

            // Extract Solution
            double[] sol = new double[canonical.Original.NumVariables];
            for (int i = 0; i < m; i++)
            {
                if (basis[i] < canonical.Original.NumVariables)
                    sol[basis[i]] = T[i + 1, n];
            }

            double zVal = T[0, n];
            if (!canonical.Original.IsMaximization) zVal = -zVal;

            log.AppendLine($"\n[RESULT] OPTIMAL SOLUTION FOUND");
            log.AppendLine($"Objective Value (z) = {zVal:F3}");

            return new Result
            {
                Status = SolutionStatus.Optimal,
                ObjectiveValue = zVal,
                VariableValues = sol,
                ExecutionLog = log.ToString()
            };
        }

        private SolutionStatus RunSimplexLoop(double[,] T, List<int> basis, int m, int n, List<string> varNames, StringBuilder log)
        {
            int iteration = 0;
            while (true)
            {
                log.AppendLine($"\n--- Iteration {iteration} ---");
                log.AppendLine(FormatTableau(T, basis, m, n, varNames));

                // 1. Pivot Column (Most negative entry in Row 0)
                int pivotCol = -1;
                double minVal = -1e-6;
                for (int j = 0; j < n; j++)
                {
                    if (T[0, j] < minVal)
                    {
                        minVal = T[0, j];
                        pivotCol = j;
                    }
                }

                if (pivotCol == -1) return SolutionStatus.Optimal; // Optimal state reached

                // 2. Pivot Row (Minimum Positive Ratio Test)
                int pivotRow = -1;
                double minRatio = double.MaxValue;
                for (int i = 1; i <= m; i++)
                {
                    if (T[i, pivotCol] > 1e-7)
                    {
                        double ratio = T[i, n] / T[i, pivotCol];
                        if (ratio < minRatio)
                        {
                            minRatio = ratio;
                            pivotRow = i;
                        }
                    }
                }

                if (pivotRow == -1) return SolutionStatus.Unbounded; // No positive entries in column

                log.AppendLine($"Entering: {varNames[pivotCol]} | Leaving: {varNames[basis[pivotRow - 1]]}");

                // 3. Perform Pivot Operation
                basis[pivotRow - 1] = pivotCol;
                double pivotVal = T[pivotRow, pivotCol];

                for (int j = 0; j <= n; j++) T[pivotRow, j] /= pivotVal;

                for (int i = 0; i <= m; i++)
                {
                    if (i != pivotRow)
                    {
                        double factor = T[i, pivotCol];
                        for (int j = 0; j <= n; j++) T[i, j] -= factor * T[pivotRow, j];
                    }
                }

                iteration++;
            }
        }

        private void ReconstructRowZero(double[,] T, CanonicalProblem c, List<int> basis, int m, int n)
        {
            for (int j = 0; j <= n; j++)
                T[0, j] = j < c.Original.NumVariables ? (c.Original.IsMaximization ? -c.Original.ObjectiveCoeffs[j] : c.Original.ObjectiveCoeffs[j]) : 0;

            for (int i = 0; i < m; i++)
            {
                int bVar = basis[i];
                double factor = T[0, bVar];
                if (Math.Abs(factor) > 1e-7)
                {
                    for (int j = 0; j <= n; j++) T[0, j] -= factor * T[i + 1, j];
                }
            }
        }

        private string FormatTableau(double[,] T, List<int> basis, int m, int n, List<string> varNames)
        {
            var sb = new StringBuilder();
            sb.Append(string.Format("{0,-8}", "Basic"));
            foreach (var v in varNames) sb.Append(string.Format("{0,10}", v));
            sb.AppendLine(string.Format("{0,10}", "RHS"));

            sb.Append(string.Format("{0,-8}", "z"));
            for (int j = 0; j <= n; j++) sb.Append(string.Format("{0,10:F3}", T[0, j]));
            sb.AppendLine();

            for (int i = 1; i <= m; i++)
            {
                sb.Append(string.Format("{0,-8}", varNames[basis[i - 1]]));
                for (int j = 0; j <= n; j++) sb.Append(string.Format("{0,10:F3}", T[i, j]));
                sb.AppendLine();
            }
            return sb.ToString();
        }

        private string FormatCanonicalText(CanonicalProblem canonical)
        {
            var sb = new StringBuilder();
            sb.AppendLine($"{(canonical.Original.IsMaximization ? "max" : "min")} z = " +
                          string.Join(" + ", canonical.VariableNames.Take(canonical.Original.NumVariables)
                          .Select((v, i) => $"{canonical.Original.ObjectiveCoeffs[i]}*{v}")));
            sb.AppendLine("Subject to:");
            for (int i = 0; i < canonical.NumConstraints; i++)
            {
                var terms = new List<string>();
                for (int j = 0; j < canonical.NumVarsTotal; j++)
                {
                    double val = canonical.TableauMatrix[i + 1, j];
                    if (Math.Abs(val) > 1e-7) terms.Add($"{val:F3}*{canonical.VariableNames[j]}");
                }
                sb.AppendLine($"  {string.Join(" + ", terms)} = {canonical.TableauMatrix[i + 1, canonical.NumVarsTotal]:F3}");
            }
            return sb.ToString();
        }
    }
}
