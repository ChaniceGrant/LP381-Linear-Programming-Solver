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

        private const double EPSILON = 1e-9;
        private const int MAX_ITERATIONS = 1000;

        public Result Solve(CanonicalProblem canonical)
        {
            if (canonical == null)
            {
                throw new ArgumentNullException(nameof(canonical));
            }

            var log = new StringBuilder();

            log.AppendLine("=== CANONICAL FORM ===");
            log.AppendLine(FormatCanonicalText(canonical));

            double[,] tableau =
                (double[,])canonical.TableauMatrix.Clone();

            int m = canonical.NumConstraints;
            int n = canonical.NumVarsTotal;

            var basis =
                new List<int>(canonical.BasicVariables);

            // ============================================================
            // PHASE 1
            // ============================================================

            if (canonical.ArtificialVarIndices.Count > 0)
            {
                log.AppendLine();
                log.AppendLine("=== PHASE 1: FIND FEASIBLE BASIS ===");

                double[,] phase1 =
                    (double[,])tableau.Clone();

                // Start Phase 1 objective row at zero.
                for (int j = 0; j <= n; j++)
                {
                    phase1[0, j] = 0.0;
                }

                /*
                 * Phase 1 maximises:
                 *
                 *       -W = -(a1 + a2 + ...)
                 *
                 * Therefore artificial variables initially have
                 * coefficient +1 in the tableau objective row.
                 */
                foreach (int artificialIndex in canonical.ArtificialVarIndices)
                {
                    phase1[0, artificialIndex] = 1.0;
                }

                /*
                 * Artificial variables are initially basic.
                 *
                 * Remove their coefficients from the objective row.
                 */
                for (int i = 0; i < m; i++)
                {
                    int basicVariable = basis[i];

                    if (!canonical.ArtificialVarIndices.Contains(basicVariable))
                    {
                        continue;
                    }

                    double factor = phase1[0, basicVariable];

                    if (Math.Abs(factor) > EPSILON)
                    {
                        for (int j = 0; j <= n; j++)
                        {
                            phase1[0, j] -=
                                factor * phase1[i + 1, j];
                        }
                    }
                }

                SolutionStatus phase1Status =
                    RunSimplexLoop(
                        phase1,
                        basis,
                        m,
                        n,
                        canonical.VariableNames,
                        log);

                /*
                 * Phase 1 objective is -W.
                 *
                 * If W = 0, the objective is 0.
                 *
                 * If W > 0, the objective is negative.
                 */
                double phase1Objective = phase1[0, n];

                if (phase1Status == SolutionStatus.Unbounded)
                {
                    log.AppendLine();
                    log.AppendLine(
                        "[RESULT] Model is INFEASIBLE.");

                    return new Result
                    {
                        Status = SolutionStatus.Infeasible,
                        ExecutionLog = log.ToString()
                    };
                }

                if (phase1Objective < -1e-7)
                {
                    log.AppendLine();
                    log.AppendLine(
                        "[RESULT] Model is INFEASIBLE " +
                        "(artificial variables cannot be driven to zero).");

                    return new Result
                    {
                        Status = SolutionStatus.Infeasible,
                        ExecutionLog = log.ToString()
                    };
                }

                // Phase 1 succeeded.
                tableau = phase1;

                log.AppendLine();
                log.AppendLine("=== PHASE 2: OPTIMIZE OBJECTIVE ===");

                ReconstructRowZero(
                    tableau,
                    canonical,
                    basis,
                    m,
                    n);
            }
            else
            {
                log.AppendLine();
                log.AppendLine("=== PRIMAL SIMPLEX ITERATIONS ===");
            }

            // ============================================================
            // PHASE 2
            // ============================================================

            SolutionStatus status =
                RunSimplexLoop(
                    tableau,
                    basis,
                    m,
                    n,
                    canonical.VariableNames,
                    log);

            if (status == SolutionStatus.Unbounded)
            {
                log.AppendLine();
                log.AppendLine("[RESULT] Model is UNBOUNDED.");

                return new Result
                {
                    Status = SolutionStatus.Unbounded,
                    ExecutionLog = log.ToString()
                };
            }

            // ============================================================
            // EXTRACT SOLUTION
            // ============================================================

            double[] solution =
                new double[canonical.Original.NumVariables];

            for (int i = 0; i < m; i++)
            {
                int basicVariable = basis[i];

                if (basicVariable >= 0 &&
                    basicVariable < canonical.Original.NumVariables)
                {
                    solution[basicVariable] =
                        tableau[i + 1, n];
                }
            }

            double objectiveValue =
                tableau[0, n];

            if (!canonical.Original.IsMaximization)
            {
                objectiveValue = -objectiveValue;
            }

            log.AppendLine();
            log.AppendLine("[RESULT] OPTIMAL SOLUTION FOUND");

            log.AppendLine(
                "Objective Value (z) = " +
                objectiveValue.ToString(
                    "F3",
                    CultureInfo.InvariantCulture));

            log.AppendLine("Variable Values:");

            for (int i = 0; i < solution.Length; i++)
            {
                log.AppendLine(
                    "  x" +
                    (i + 1) +
                    " = " +
                    solution[i].ToString(
                        "F3",
                        CultureInfo.InvariantCulture));
            }

            return new Result
            {
                Status = SolutionStatus.Optimal,
                ObjectiveValue = objectiveValue,
                VariableValues = solution,
                ExecutionLog = log.ToString()
            };
        }

        // ================================================================
        // SIMPLEX LOOP
        // ================================================================

        private SolutionStatus RunSimplexLoop(
            double[,] tableau,
            List<int> basis,
            int m,
            int n,
            List<string> variableNames,
            StringBuilder log)
        {
            int iteration = 0;

            while (true)
            {
                if (iteration >= MAX_ITERATIONS)
                {
                    log.AppendLine();
                    log.AppendLine(
                        "[ERROR] Maximum simplex iterations reached.");

                    return SolutionStatus.Unbounded;
                }

                log.AppendLine();
                log.AppendLine(
                    "--- Iteration " +
                    iteration +
                    " ---");

                log.AppendLine(
                    FormatTableau(
                        tableau,
                        basis,
                        m,
                        n,
                        variableNames));

                // --------------------------------------------------------
                // Select entering variable.
                // --------------------------------------------------------

                int pivotColumn = -1;
                double mostNegative = -EPSILON;

                for (int j = 0; j < n; j++)
                {
                    if (tableau[0, j] < mostNegative)
                    {
                        mostNegative = tableau[0, j];
                        pivotColumn = j;
                    }
                }

                // No negative reduced cost = optimal.
                if (pivotColumn == -1)
                {
                    return SolutionStatus.Optimal;
                }

                // --------------------------------------------------------
                // Select leaving variable.
                // --------------------------------------------------------

                int pivotRow = -1;
                double minimumRatio =
                    double.PositiveInfinity;

                for (int i = 1; i <= m; i++)
                {
                    double coefficient =
                        tableau[i, pivotColumn];

                    if (coefficient > EPSILON)
                    {
                        double rhs =
                            tableau[i, n];

                        double ratio =
                            rhs / coefficient;

                        if (ratio >= -EPSILON &&
                            ratio < minimumRatio)
                        {
                            minimumRatio = ratio;
                            pivotRow = i;
                        }
                    }
                }

                // No valid leaving variable = unbounded.
                if (pivotRow == -1)
                {
                    log.AppendLine();
                    log.AppendLine(
                        "Entering variable: " +
                        variableNames[pivotColumn]);

                    return SolutionStatus.Unbounded;
                }

                log.AppendLine(
                    "Entering: " +
                    variableNames[pivotColumn] +
                    " | Leaving: " +
                    variableNames[basis[pivotRow - 1]]);

                // --------------------------------------------------------
                // Pivot.
                // --------------------------------------------------------

                double pivotValue =
                    tableau[pivotRow, pivotColumn];

                if (Math.Abs(pivotValue) < EPSILON)
                {
                    return SolutionStatus.Unbounded;
                }

                basis[pivotRow - 1] =
                    pivotColumn;

                // Divide pivot row by pivot value.
                for (int j = 0; j <= n; j++)
                {
                    tableau[pivotRow, j] /=
                        pivotValue;
                }

                // Eliminate pivot column from all other rows.
                for (int i = 0; i <= m; i++)
                {
                    if (i == pivotRow)
                    {
                        continue;
                    }

                    double factor =
                        tableau[i, pivotColumn];

                    if (Math.Abs(factor) < EPSILON)
                    {
                        continue;
                    }

                    for (int j = 0; j <= n; j++)
                    {
                        tableau[i, j] -=
                            factor *
                            tableau[pivotRow, j];
                    }
                }

                CleanTableau(
                    tableau,
                    m,
                    n);

                iteration++;
            }
        }

        // ================================================================
        // REBUILD OBJECTIVE ROW FOR PHASE 2
        // ================================================================

        private void ReconstructRowZero(
            double[,] tableau,
            CanonicalProblem canonical,
            List<int> basis,
            int m,
            int n)
        {
            // Clear objective row.
            for (int j = 0; j <= n; j++)
            {
                tableau[0, j] = 0.0;
            }

            // Original objective coefficients.
            for (int j = 0;
                 j < canonical.Original.NumVariables;
                 j++)
            {
                if (canonical.Original.IsMaximization)
                {
                    tableau[0, j] =
                        -canonical.Original.ObjectiveCoeffs[j];
                }
                else
                {
                    tableau[0, j] =
                        canonical.Original.ObjectiveCoeffs[j];
                }
            }

            tableau[0, n] = 0.0;

            /*
             * Eliminate all basic-variable coefficients from Row 0.
             */
            for (int i = 0; i < m; i++)
            {
                int basicVariable =
                    basis[i];

                if (basicVariable < 0 ||
                    basicVariable >= n)
                {
                    continue;
                }

                double factor =
                    tableau[0, basicVariable];

                if (Math.Abs(factor) < EPSILON)
                {
                    continue;
                }

                for (int j = 0; j <= n; j++)
                {
                    tableau[0, j] -=
                        factor *
                        tableau[i + 1, j];
                }
            }

            CleanTableau(
                tableau,
                m,
                n);
        }

        // ================================================================
        // CLEAN SMALL FLOATING POINT VALUES
        // ================================================================

        private void CleanTableau(
            double[,] tableau,
            int m,
            int n)
        {
            for (int i = 0; i <= m; i++)
            {
                for (int j = 0; j <= n; j++)
                {
                    if (Math.Abs(tableau[i, j]) < EPSILON)
                    {
                        tableau[i, j] = 0.0;
                    }
                }
            }
        }

        // ================================================================
        // FORMAT TABLEAU
        // ================================================================

        private string FormatTableau(
            double[,] tableau,
            List<int> basis,
            int m,
            int n,
            List<string> variableNames)
        {
            var sb =
                new StringBuilder();

            sb.Append(
                string.Format(
                    CultureInfo.InvariantCulture,
                    "{0,-10}",
                    "Basic"));

            foreach (string variable in variableNames)
            {
                sb.Append(
                    string.Format(
                        CultureInfo.InvariantCulture,
                        "{0,10}",
                        variable));
            }

            sb.AppendLine(
                string.Format(
                    CultureInfo.InvariantCulture,
                    "{0,12}",
                    "RHS"));

            // Objective row.
            sb.Append(
                string.Format(
                    CultureInfo.InvariantCulture,
                    "{0,-10}",
                    "z"));

            for (int j = 0; j <= n; j++)
            {
                sb.Append(
                    string.Format(
                        CultureInfo.InvariantCulture,
                        "{0,10:F3}",
                        tableau[0, j]));
            }

            sb.AppendLine();

            // Constraint rows.
            for (int i = 1; i <= m; i++)
            {
                string basicName =
                    "Unknown";

                int basicIndex =
                    basis[i - 1];

                if (basicIndex >= 0 &&
                    basicIndex < variableNames.Count)
                {
                    basicName =
                        variableNames[basicIndex];
                }

                sb.Append(
                    string.Format(
                        CultureInfo.InvariantCulture,
                        "{0,-10}",
                        basicName));

                for (int j = 0; j <= n; j++)
                {
                    sb.Append(
                        string.Format(
                            CultureInfo.InvariantCulture,
                            "{0,10:F3}",
                            tableau[i, j]));
                }

                sb.AppendLine();
            }

            return sb.ToString();
        }

        // ================================================================
        // FORMAT CANONICAL FORM
        // ================================================================

        private string FormatCanonicalText(
            CanonicalProblem canonical)
        {
            var sb =
                new StringBuilder();

            string objectiveDirection =
                canonical.Original.IsMaximization
                    ? "max"
                    : "min";

            var objectiveTerms =
                new List<string>();

            for (int i = 0;
                 i < canonical.Original.NumVariables;
                 i++)
            {
                double coefficient =
                    canonical.Original.ObjectiveCoeffs[i];

                string variable =
                    canonical.VariableNames[i];

                if (coefficient >= 0)
                {
                    objectiveTerms.Add(
                        coefficient.ToString(
                            "F3",
                            CultureInfo.InvariantCulture) +
                        "*" +
                        variable);
                }
                else
                {
                    objectiveTerms.Add(
                        "- " +
                        Math.Abs(coefficient).ToString(
                            "F3",
                            CultureInfo.InvariantCulture) +
                        "*" +
                        variable);
                }
            }

            sb.AppendLine(
                objectiveDirection +
                " z = " +
                string.Join(
                    " + ",
                    objectiveTerms));

            sb.AppendLine("Subject to:");

            for (int i = 0;
                 i < canonical.NumConstraints;
                 i++)
            {
                var terms =
                    new List<string>();

                for (int j = 0;
                     j < canonical.NumVarsTotal;
                     j++)
                {
                    double value =
                        canonical.TableauMatrix[
                            i + 1,
                            j];

                    if (Math.Abs(value) <= EPSILON)
                    {
                        continue;
                    }

                    string variable =
                        canonical.VariableNames[j];

                    string formattedValue =
                        Math.Abs(value).ToString(
                            "F3",
                            CultureInfo.InvariantCulture);

                    if (value >= 0)
                    {
                        terms.Add(
                            formattedValue +
                            "*" +
                            variable);
                    }
                    else
                    {
                        terms.Add(
                            "- " +
                            formattedValue +
                            "*" +
                            variable);
                    }
                }

                double rhs =
                    canonical.TableauMatrix[
                        i + 1,
                        canonical.NumVarsTotal];

                sb.AppendLine(
                    "  " +
                    string.Join(
                        " + ",
                        terms) +
                    " = " +
                    rhs.ToString(
                        "F3",
                        CultureInfo.InvariantCulture));
            }

            return sb.ToString();
        }
    }
}