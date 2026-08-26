using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

using LPR381Solver.Models;

namespace LPR381Solver.Algorithms
{

    public class CuttingPlane
    {
        private const double Epsilon = 1e-9;
        private const int MaximumCuts = 30;

        private SimplexCore _simplex = new SimplexCore();
        public int CutCount { get; private set; }

        public double ObjectiveValue { get; private set; }

        public double[] Solution { get; private set; } = Array.Empty<double>();

        public bool Solved { get; private set; }

        private readonly StringBuilder _log = new StringBuilder();

        public string Log => _log.ToString() + _simplex.Log;

        public void Solve(LPModel model)
        {
            ValidateIntegerModel(model);

            _simplex = new SimplexCore();
            _simplex.WriteLine("==========================================================");
            _simplex.WriteLine("   CUTTING PLANE ALGORITHM (Gomory fractional cuts)");
            _simplex.WriteLine("==========================================================");
            _simplex.WriteLine(string.Empty);

            _simplex.BuildCanonicalForm(model);
            _simplex.WriteLine("STEP 1: Solve the LP relaxation with the Primal Simplex Algorithm");
            _simplex.WriteLine("----------------------------------------------------------");

            if (!_simplex.SolvePrimal())
            {
                _simplex.WriteLine("The LP relaxation is unbounded, so the integer model cannot be solved.");
                Solved = false;
                return;
            }

            _simplex.WriteLine(string.Format(CultureInfo.InvariantCulture,
                "LP relaxation optimum: z = {0:F3}", _simplex.ObjectiveValue));
            _simplex.WriteLine(string.Empty);

            CutCount = 0;

            while (CutCount < MaximumCuts)
            {
                int sourceRow = FindFractionalRow();

                if (sourceRow == -1)
                {
                    _simplex.WriteLine("STEP 2: All basic variables are integer. The solution is optimal.");
                    _simplex.WriteLine(string.Empty);
                    Finish();
                    return;
                }

                CutCount++;
                string cutName = "c" + CutCount;

                double sourceRhs = _simplex.Tableau[sourceRow][_simplex.ColumnCount];
                string basicName = _simplex.ColumnNames[_simplex.Basis[sourceRow - 1]];

                _simplex.WriteLine("----------------------------------------------------------");
                _simplex.WriteLine(string.Format(CultureInfo.InvariantCulture,
                    "CUT {0}: source row {1}, basic variable {2} = {3:F3} is fractional",
                    CutCount, sourceRow, basicName, sourceRhs));

                int columnCount = _simplex.ColumnCount;
                double[] cutCoefficients = new double[columnCount];
                StringBuilder cutText = new StringBuilder();

                for (int j = 0; j < columnCount; j++)
                {
                    double fractionalPart = FractionalPart(_simplex.Tableau[sourceRow][j]);
                    cutCoefficients[j] = -fractionalPart;

                    if (fractionalPart > Epsilon)
                    {
                        if (cutText.Length > 0) cutText.Append(" + ");
                        cutText.Append(string.Format(CultureInfo.InvariantCulture,
                            "{0:F3}{1}", fractionalPart, _simplex.ColumnNames[j]));
                    }
                }

                double cutRhs = -FractionalPart(sourceRhs);

                _simplex.WriteLine(string.Format(CultureInfo.InvariantCulture,
                    "  Cut: {0} >= {1:F3}", cutText.ToString(), FractionalPart(sourceRhs)));
                _simplex.WriteLine(string.Format(CultureInfo.InvariantCulture,
                    "  Added to the tableau as: -({0}) + {1} = {2:F3}",
                    cutText.ToString(), cutName, cutRhs));
                _simplex.WriteLine(string.Empty);

                _simplex.AppendCutRow(cutCoefficients, cutRhs, cutName);
                _simplex.WriteTableau("after-" + cutName);

                _simplex.WriteLine("  Restoring feasibility with the Dual Simplex Algorithm:");

                if (!_simplex.SolveDual(cutName))
                {
                    _simplex.WriteLine("The model became infeasible after adding the cut.");
                    Solved = false;
                    return;
                }

                _simplex.WriteLine(string.Format(CultureInfo.InvariantCulture,
                    "  Objective after cut {0}: z = {1:F3}", CutCount, _simplex.ObjectiveValue));
                _simplex.WriteLine(string.Empty);
            }

            _simplex.WriteLine(string.Format(CultureInfo.InvariantCulture,
                "Stopped after {0} cuts without reaching an all integer solution.", MaximumCuts));
            Solved = false;
        }

        private int FindFractionalRow()
        {
            for (int i = 1; i < _simplex.Tableau.Count; i++)
            {
                if (_simplex.Basis[i - 1] >= _simplex.DecisionVariableCount)
                    continue; 

                if (FractionalPart(_simplex.Tableau[i][_simplex.ColumnCount]) > Epsilon)
                    return i;
            }

            return -1;
        }

        private static double FractionalPart(double value)
        {
            double fraction = value - Math.Floor(value);

            if (fraction < Epsilon || fraction > 1.0 - Epsilon)
                return 0.0;

            return fraction;
        }

        private void Finish()
        {
            Solution = _simplex.GetSolution();
            ObjectiveValue = _simplex.ObjectiveValue;
            Solved = true;

            _simplex.WriteLine("==========================================================");
            _simplex.WriteLine("   INTEGER OPTIMAL SOLUTION");
            _simplex.WriteLine("==========================================================");

            for (int j = 0; j < Solution.Length; j++)
            {
                _simplex.WriteLine(string.Format(CultureInfo.InvariantCulture,
                    "  {0} = {1:F3}", _simplex.ColumnNames[j], Solution[j]));
            }

            _simplex.WriteLine(string.Format(CultureInfo.InvariantCulture,
                "  Objective value z = {0:F3}", ObjectiveValue));
            _simplex.WriteLine(string.Format(CultureInfo.InvariantCulture,
                "  Cuts generated = {0}", CutCount));
        }

        private static void ValidateIntegerModel(LPModel model)
        {
            if (model == null)
                throw new ArgumentNullException(nameof(model));

            if (model.Variables == null || model.Variables.Count == 0)
                throw new InvalidOperationException("The model has no decision variables.");

            bool hasIntegerVariable = false;

            foreach (Variable variable in model.Variables)
            {
                if (string.Equals(variable.SignRestriction, "int", StringComparison.OrdinalIgnoreCase))
                {
                    hasIntegerVariable = true;
                }
                else if (string.Equals(variable.SignRestriction, "bin", StringComparison.OrdinalIgnoreCase))
                {
                    throw new InvalidOperationException(
                        "This model is binary. Use the Branch and Bound Knapsack algorithm instead.");
                }
                else if (string.Equals(variable.SignRestriction, "urs", StringComparison.OrdinalIgnoreCase))
                {
                    throw new InvalidOperationException(
                        "Unrestricted in sign variables are not supported by this Cutting Plane implementation.");
                }
            }

            if (!hasIntegerVariable)
            {
                throw new InvalidOperationException(
                    "No integer variables were found. Solve this model with the Primal Simplex Algorithm instead.");
            }
        }
    }
}
