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

        public void BuildCanonicalForm(LPModel model)
        {
            Validate(model);

            DecisionVariableCount = model.Variables.Count;
            WasMinimisation = string.Equals(model.ObjectiveType, "min",
                                            StringComparison.OrdinalIgnoreCase);

            int constraintCount = model.Constraints.Count;
            int totalColumns = DecisionVariableCount + constraintCount;

            ColumnNames = new List<string>();
            for (int j = 0; j < DecisionVariableCount; j++)
                ColumnNames.Add(model.Variables[j].Name);
            for (int i = 0; i < constraintCount; i++)
                ColumnNames.Add("s" + (i + 1));

            Tableau = new List<double[]>();
            Basis = new List<int>();

            double[] objectiveRow = new double[totalColumns + 1];
            for (int j = 0; j < DecisionVariableCount; j++)
            {
                double coefficient = model.Variables[j].ObjectiveCoefficient;
                if (WasMinimisation) coefficient = -coefficient;
                objectiveRow[j] = -coefficient;
            }
            Tableau.Add(objectiveRow);

            for (int i = 0; i < constraintCount; i++)
            {
                Constraint constraint = model.Constraints[i];
                double[] row = new double[totalColumns + 1];

                for (int j = 0; j < DecisionVariableCount; j++)
                    row[j] = constraint.Coefficients[j];

                row[DecisionVariableCount + i] = 1.0;      
                row[totalColumns] = constraint.RightHandSide;

                Tableau.Add(row);
                Basis.Add(DecisionVariableCount + i);
            }

            WriteCanonicalForm(model);
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

        private void Validate(LPModel model)
        {
            if (model == null)
                throw new ArgumentNullException(nameof(model));

            if (model.Variables == null || model.Variables.Count == 0)
                throw new InvalidOperationException("The model has no decision variables.");

            if (model.Constraints == null || model.Constraints.Count == 0)
                throw new InvalidOperationException("The model has no constraints.");

            foreach (Constraint constraint in model.Constraints)
            {
                if (constraint.Relation != "<=")
                    throw new InvalidOperationException(
                        "This simplex engine supports <= constraints only. "
                        + "Found the relation '" + constraint.Relation + "'.");

                if (constraint.RightHandSide < 0)
                    throw new InvalidOperationException(
                        "This simplex engine requires non-negative right hand sides.");

                if (constraint.Coefficients.Count != model.Variables.Count)
                    throw new InvalidOperationException(
                        "A constraint has a different number of coefficients than there are variables.");
            }
        }

        private void WriteCanonicalForm(LPModel model)
        {
            _log.AppendLine("CANONICAL FORM");
            _log.AppendLine("----------------------------------------------------------");

            _log.Append(WasMinimisation
                ? "  min z = "
                : "  max z = ");

            for (int j = 0; j < DecisionVariableCount; j++)
            {
                double coefficient = model.Variables[j].ObjectiveCoefficient;
                _log.Append(string.Format(CultureInfo.InvariantCulture,
                    "{0}{1:F3}{2} ", coefficient >= 0 ? "+" : "-",
                    Math.Abs(coefficient), model.Variables[j].Name));
            }
            _log.AppendLine();

            if (WasMinimisation)
                _log.AppendLine("  (solved as a maximisation of the negated objective)");

            for (int i = 0; i < model.Constraints.Count; i++)
            {
                Constraint constraint = model.Constraints[i];
                _log.Append("  ");

                for (int j = 0; j < DecisionVariableCount; j++)
                {
                    _log.Append(string.Format(CultureInfo.InvariantCulture,
                        "{0}{1:F3}{2} ", constraint.Coefficients[j] >= 0 ? "+" : "-",
                        Math.Abs(constraint.Coefficients[j]), model.Variables[j].Name));
                }

                _log.AppendLine(string.Format(CultureInfo.InvariantCulture,
                    "+ s{0} = {1:F3}", i + 1, constraint.RightHandSide));
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
