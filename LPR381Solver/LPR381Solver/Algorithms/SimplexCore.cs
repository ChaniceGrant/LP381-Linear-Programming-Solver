using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

using LPR381Solver.Models;

namespace LPR381Solver.Algorithms
{
    public class SimplexCore
    {
        public const double Epsilon = 1e-9;

        public List<double[]> Tableau { get; private set; } = new List<double[]>();

        public List<string> ColumnNames { get; private set; } = new List<string>();

        public List<int> Basis { get; private set; } = new List<int>();

        public int DecisionVariableCount { get; private set; }

        public bool WasMinimisation { get; private set; }

        public bool IsUnbounded { get; private set; }

        public bool IsInfeasible { get; private set; }

        private readonly StringBuilder _log = new StringBuilder();

        public string Log => _log.ToString();

        public int ColumnCount => ColumnNames.Count;

        public double ObjectiveValue
        {
            get
            {
                double z = Tableau[0][ColumnCount];
                return WasMinimisation ? -z : z;
            }
        }

        public void BuildCanonicalForm(LpProblem problem)
        {
            Validate(problem);

            DecisionVariableCount = problem.NumVariables;
            WasMinimisation = !problem.IsMaximization;

            int constraintCount = problem.NumConstraints;
            int totalColumns = DecisionVariableCount + constraintCount;

            ColumnNames = new List<string>();
            for (int j = 0; j < DecisionVariableCount; j++)
                ColumnNames.Add("x" + (j + 1));
            for (int i = 0; i < constraintCount; i++)
                ColumnNames.Add("s" + (i + 1));

            Tableau = new List<double[]>();
            Basis = new List<int>();

            double[] objectiveRow = new double[totalColumns + 1];
            for (int j = 0; j < DecisionVariableCount; j++)
            {
                double coefficient = problem.ObjectiveCoeffs[j];
                if (WasMinimisation) coefficient = -coefficient;
                objectiveRow[j] = -coefficient;
            }
            Tableau.Add(objectiveRow);

            for (int i = 0; i < constraintCount; i++)
            {
                List<double> coefficients = problem.ConstraintCoeffs[i];
                double[] row = new double[totalColumns + 1];

                for (int j = 0; j < DecisionVariableCount; j++)
                    row[j] = coefficients[j];

                row[DecisionVariableCount + i] = 1.0;
                row[totalColumns] = problem.Rhs[i];

                Tableau.Add(row);
                Basis.Add(DecisionVariableCount + i);
            }

            WriteCanonicalForm(problem);
        }

        public bool SolvePrimal()
        {
            int iteration = 1;

            while (true)
            {
                WriteTableau("t-" + iteration);

                int pivotColumn = -1;
                double mostNegative = -Epsilon;

                for (int j = 0; j < ColumnCount; j++)
                {
                    if (Tableau[0][j] < mostNegative)
                    {
                        mostNegative = Tableau[0][j];
                        pivotColumn = j;
                    }
                }

                if (pivotColumn == -1)
                {
                    _log.AppendLine("  All z row coefficients are non-negative. Optimal.");
                    _log.AppendLine();
                    return true;
                }

                int pivotRow = -1;
                double bestRatio = 0.0;

                for (int i = 1; i < Tableau.Count; i++)
                {
                    if (Tableau[i][pivotColumn] > Epsilon)
                    {
                        double ratio = Tableau[i][ColumnCount] / Tableau[i][pivotColumn];
                        if (pivotRow == -1 || ratio < bestRatio - Epsilon)
                        {
                            bestRatio = ratio;
                            pivotRow = i;
                        }
                    }
                }

                if (pivotRow == -1)
                {
                    IsUnbounded = true;
                    _log.AppendLine("  No positive entry in the entering column. The model is UNBOUNDED.");
                    _log.AppendLine();
                    return false;
                }

                _log.AppendLine(string.Format(CultureInfo.InvariantCulture,
                    "  Pivot: entering {0}, leaving {1} (row {2}), ratio {3:F3}",
                    ColumnNames[pivotColumn], ColumnNames[Basis[pivotRow - 1]],
                    pivotRow, bestRatio));
                _log.AppendLine();

                Pivot(pivotRow, pivotColumn);
                Basis[pivotRow - 1] = pivotColumn;
                iteration++;

                if (iteration > 200)
                    throw new InvalidOperationException("Primal Simplex did not converge.");
            }
        }

        public bool SolveDual(string labelPrefix)
        {
            int iteration = 1;

            while (true)
            {
                int pivotRow = -1;
                double mostNegative = -Epsilon;

                for (int i = 1; i < Tableau.Count; i++)
                {
                    if (Tableau[i][ColumnCount] < mostNegative)
                    {
                        mostNegative = Tableau[i][ColumnCount];
                        pivotRow = i;
                    }
                }

                if (pivotRow == -1)
                {
                    _log.AppendLine("  All right hand sides are non-negative. Feasible again.");
                    _log.AppendLine();
                    return true;
                }

                int pivotColumn = -1;
                double bestRatio = 0.0;

                for (int j = 0; j < ColumnCount; j++)
                {
                    if (Tableau[pivotRow][j] < -Epsilon)
                    {
                        double ratio = Math.Abs(Tableau[0][j] / Tableau[pivotRow][j]);
                        if (pivotColumn == -1 || ratio < bestRatio - Epsilon)
                        {
                            bestRatio = ratio;
                            pivotColumn = j;
                        }
                    }
                }

                if (pivotColumn == -1)
                {
                    IsInfeasible = true;
                    _log.AppendLine("  No negative entry in the leaving row. The model is INFEASIBLE.");
                    _log.AppendLine();
                    return false;
                }

                _log.AppendLine(string.Format(CultureInfo.InvariantCulture,
                    "  Dual pivot: leaving {0} (row {1}), entering {2}",
                    ColumnNames[Basis[pivotRow - 1]], pivotRow, ColumnNames[pivotColumn]));

                Pivot(pivotRow, pivotColumn);
                Basis[pivotRow - 1] = pivotColumn;

                WriteTableau(labelPrefix + "-dual-" + iteration);
                iteration++;

                if (iteration > 200)
                    throw new InvalidOperationException("Dual Simplex did not converge.");
            }
        }

        public void Pivot(int pivotRow, int pivotColumn)
        {
            double pivotValue = Tableau[pivotRow][pivotColumn];

            for (int j = 0; j <= ColumnCount; j++)
                Tableau[pivotRow][j] /= pivotValue;

            for (int i = 0; i < Tableau.Count; i++)
            {
                if (i == pivotRow) continue;

                double factor = Tableau[i][pivotColumn];
                if (Math.Abs(factor) < Epsilon) continue;

                for (int j = 0; j <= ColumnCount; j++)
                    Tableau[i][j] -= factor * Tableau[pivotRow][j];
            }
        }

        public void AppendCutRow(double[] cutCoefficients, double cutRightHandSide, string cutName)
        {
            for (int i = 0; i < Tableau.Count; i++)
            {
                double[] widened = new double[ColumnCount + 2];
                Array.Copy(Tableau[i], widened, ColumnCount);
                widened[ColumnCount] = 0.0;
                widened[ColumnCount + 1] = Tableau[i][ColumnCount];
                Tableau[i] = widened;
            }

            ColumnNames.Add(cutName);

            double[] newRow = new double[ColumnCount + 1];
            for (int j = 0; j < cutCoefficients.Length; j++)
                newRow[j] = cutCoefficients[j];
            newRow[ColumnCount - 1] = 1.0;
            newRow[ColumnCount] = cutRightHandSide;

            Tableau.Add(newRow);
            Basis.Add(ColumnCount - 1);
        }

        public double[] GetSolution()
        {
            double[] solution = new double[DecisionVariableCount];

            for (int i = 0; i < Basis.Count; i++)
            {
                if (Basis[i] < DecisionVariableCount)
                    solution[Basis[i]] = Tableau[i + 1][ColumnCount];
            }

            return solution;
        }

        private void Validate(LpProblem problem)
        {
            if (problem == null)
                throw new ArgumentNullException(nameof(problem));

            if (problem.NumVariables == 0)
                throw new InvalidOperationException("The model has no decision variables.");

            if (problem.NumConstraints == 0)
                throw new InvalidOperationException("The model has no constraints.");

            for (int i = 0; i < problem.NumConstraints; i++)
            {
                if (problem.Relations[i] != "<=")
                    throw new InvalidOperationException(
                        "This Cutting Plane implementation supports <= constraints only. "
                        + "Constraint " + (i + 1) + " uses '" + problem.Relations[i] + "'. "
                        + "Solve this model with the Primal Simplex Algorithm instead.");

                if (problem.Rhs[i] < 0)
                    throw new InvalidOperationException(
                        "This Cutting Plane implementation requires non-negative right hand sides.");

                if (problem.ConstraintCoeffs[i].Count != problem.NumVariables)
                    throw new InvalidOperationException(
                        "Constraint " + (i + 1) + " has a different number of coefficients "
                        + "than there are variables.");
            }
        }

        private void WriteCanonicalForm(LpProblem problem)
        {
            _log.AppendLine("CANONICAL FORM");
            _log.AppendLine("----------------------------------------------------------");

            _log.Append(problem.IsMaximization ? "  max z = " : "  min z = ");

            for (int j = 0; j < DecisionVariableCount; j++)
            {
                double coefficient = problem.ObjectiveCoeffs[j];
                _log.Append(string.Format(CultureInfo.InvariantCulture,
                    "{0}{1:F3}x{2} ", coefficient >= 0 ? "+" : "-",
                    Math.Abs(coefficient), j + 1));
            }
            _log.AppendLine();

            if (WasMinimisation)
                _log.AppendLine("  (solved as a maximisation of the negated objective)");

            for (int i = 0; i < problem.NumConstraints; i++)
            {
                List<double> coefficients = problem.ConstraintCoeffs[i];
                _log.Append("  ");

                for (int j = 0; j < DecisionVariableCount; j++)
                {
                    _log.Append(string.Format(CultureInfo.InvariantCulture,
                        "{0}{1:F3}x{2} ", coefficients[j] >= 0 ? "+" : "-",
                        Math.Abs(coefficients[j]), j + 1));
                }

                _log.AppendLine(string.Format(CultureInfo.InvariantCulture,
                    "+ s{0} = {1:F3}", i + 1, problem.Rhs[i]));
            }

            _log.AppendLine();
        }

        public void WriteTableau(string label)
        {
            _log.Append("  " + label.PadRight(12));
            foreach (string name in ColumnNames)
                _log.Append(name.PadLeft(9));
            _log.AppendLine("rhs".PadLeft(10));

            _log.Append("  " + "z".PadRight(12));
            for (int j = 0; j < ColumnCount; j++)
                _log.Append(Tableau[0][j].ToString("F3", CultureInfo.InvariantCulture).PadLeft(9));
            _log.AppendLine(Tableau[0][ColumnCount].ToString("F3", CultureInfo.InvariantCulture).PadLeft(10));

            for (int i = 1; i < Tableau.Count; i++)
            {
                string rowLabel = i + " (" + ColumnNames[Basis[i - 1]] + ")";
                _log.Append("  " + rowLabel.PadRight(12));

                for (int j = 0; j < ColumnCount; j++)
                    _log.Append(Tableau[i][j].ToString("F3", CultureInfo.InvariantCulture).PadLeft(9));

                _log.AppendLine(Tableau[i][ColumnCount].ToString("F3", CultureInfo.InvariantCulture).PadLeft(10));
            }

            _log.AppendLine();
        }

        public void WriteLine(string text)
        {
            _log.AppendLine(text);
        }
    }
}