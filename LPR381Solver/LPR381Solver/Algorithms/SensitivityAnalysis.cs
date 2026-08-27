using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using LPR381Solver.Models;

namespace LPR381Solver.Services
{
    /// <summary>
    /// Post-optimal sensitivity operations performed on an optimal simplex basis.
    /// Ranges reported by this class are the ranges for which the CURRENT basis
    /// remains feasible/optimal. Applying a change re-solves the modified model.
    /// </summary>
    public sealed class SensitivityAnalysisService
    {
        private const double Epsilon = 1e-9;

        public sealed class RangeResult
        {
            public string ItemName { get; init; } = string.Empty;
            public double CurrentValue { get; init; }
            public double LowerBound { get; init; } = double.NegativeInfinity;
            public double UpperBound { get; init; } = double.PositiveInfinity;
            public string Explanation { get; init; } = string.Empty;

            public override string ToString()
            {
                return $"{ItemName}\n" +
                       $"Current value : {F3(CurrentValue)}\n" +
                       $"Allowable range: {FormatBound(LowerBound)} <= value <= {FormatBound(UpperBound)}\n" +
                       Explanation;
            }
        }

        public sealed class ChangeResult
        {
            public LpProblem ModifiedProblem { get; init; } = new();
            public PrimalSimplexSolver.Result SolverResult { get; init; } = new();
            public string Description { get; init; } = string.Empty;
        }

        public sealed class ShadowPriceResult
        {
            public int ConstraintNumber { get; init; }
            public double ShadowPrice { get; init; }
        }

        public RangeResult GetNonBasicVariableObjectiveRange(
            PrimalSimplexSolver.Result result,
            int originalVariableIndex)
        {
            EnsureOptimal(result);
            EnsureVariableIndex(result, originalVariableIndex);
            EnsureVariableStatus(result, originalVariableIndex, mustBeBasic: false);
            return GetObjectiveCoefficientRangeCore(result, originalVariableIndex, "Non-Basic Variable");
        }

        public RangeResult GetBasicVariableObjectiveRange(
            PrimalSimplexSolver.Result result,
            int originalVariableIndex)
        {
            EnsureOptimal(result);
            EnsureVariableIndex(result, originalVariableIndex);
            EnsureVariableStatus(result, originalVariableIndex, mustBeBasic: true);
            return GetObjectiveCoefficientRangeCore(result, originalVariableIndex, "Basic Variable");
        }

        public RangeResult GetRhsRange(
            PrimalSimplexSolver.Result result,
            int originalConstraintIndex)
        {
            EnsureOptimal(result);
            CanonicalProblem canonical = RequireCanonical(result);
            LpProblem lp = canonical.Original;

            if (originalConstraintIndex < 0 || originalConstraintIndex >= lp.NumConstraints)
                throw new ArgumentOutOfRangeException(nameof(originalConstraintIndex));

            double[,] inverse = GetBasisInverse(result);
            double current = lp.Rhs[originalConstraintIndex];
            double rowMultiplier = current < 0.0 ? -1.0 : 1.0;

            double lowerDelta = double.NegativeInfinity;
            double upperDelta = double.PositiveInfinity;
            int rhsColumn = canonical.NumVarsTotal;

            for (int i = 0; i < canonical.NumConstraints; i++)
            {
                double basicValue = result.FinalTableau[i + 1, rhsColumn];
                double slope = inverse[i, originalConstraintIndex] * rowMultiplier;
                IntersectNonNegative(ref lowerDelta, ref upperDelta, basicValue, slope);
            }

            return new RangeResult
            {
                ItemName = $"Constraint {originalConstraintIndex + 1} RHS",
                CurrentValue = current,
                LowerBound = AddBound(current, lowerDelta),
                UpperBound = AddBound(current, upperDelta),
                Explanation = "Within this interval the current basis remains primal feasible."
            };
        }

        public RangeResult GetNonBasicColumnCoefficientRange(
            PrimalSimplexSolver.Result result,
            int originalConstraintIndex,
            int originalVariableIndex)
        {
            EnsureOptimal(result);
            CanonicalProblem canonical = RequireCanonical(result);
            LpProblem lp = canonical.Original;

            if (originalConstraintIndex < 0 || originalConstraintIndex >= lp.NumConstraints)
                throw new ArgumentOutOfRangeException(nameof(originalConstraintIndex));
            EnsureVariableIndex(result, originalVariableIndex);

            List<int> columns = canonical.OriginalVariableColumns[originalVariableIndex];
            if (columns.Count != 1)
                throw new InvalidOperationException(
                    "This operation requires a variable represented by one canonical column. " +
                    "Unrestricted variables are represented by two columns.");

            int column = columns[0];
            if (result.FinalBasis.Contains(column))
                throw new InvalidOperationException($"x{originalVariableIndex + 1} is basic, not non-basic.");

            double reducedCost = result.FinalTableau[0, column];
            double[] pricesEffective = GetEffectiveShadowPrices(result);
            double rowMultiplier = lp.Rhs[originalConstraintIndex] < 0.0 ? -1.0 : 1.0;
            double variableMultiplier = canonical.OriginalVariableMultipliers[originalVariableIndex][0];
            double slope = pricesEffective[originalConstraintIndex] * rowMultiplier * variableMultiplier;

            double lowerDelta = double.NegativeInfinity;
            double upperDelta = double.PositiveInfinity;
            IntersectNonNegative(ref lowerDelta, ref upperDelta, reducedCost, slope);

            double current = lp.ConstraintCoeffs[originalConstraintIndex][originalVariableIndex];
            return new RangeResult
            {
                ItemName = $"Technological coefficient a[{originalConstraintIndex + 1},{originalVariableIndex + 1}] in non-basic column x{originalVariableIndex + 1}",
                CurrentValue = current,
                LowerBound = AddBound(current, lowerDelta),
                UpperBound = AddBound(current, upperDelta),
                Explanation = "Within this interval the reduced cost of the selected non-basic column remains optimal."
            };
        }

        public IReadOnlyList<ShadowPriceResult> GetShadowPrices(PrimalSimplexSolver.Result result)
        {
            EnsureOptimal(result);
            CanonicalProblem canonical = RequireCanonical(result);
            LpProblem lp = canonical.Original;
            double[] effective = GetEffectiveShadowPrices(result);
            double objectiveMultiplier = lp.IsMaximization ? 1.0 : -1.0;
            var prices = new List<ShadowPriceResult>();

            for (int i = 0; i < lp.NumConstraints; i++)
            {
                double rowMultiplier = lp.Rhs[i] < 0.0 ? -1.0 : 1.0;
                prices.Add(new ShadowPriceResult
                {
                    ConstraintNumber = i + 1,
                    ShadowPrice = objectiveMultiplier * effective[i] * rowMultiplier
                });
            }

            return prices;
        }

        public ChangeResult ApplyObjectiveCoefficientChange(
            PrimalSimplexSolver.Result currentResult,
            int originalVariableIndex,
            double newCoefficient)
        {
            EnsureOptimal(currentResult);
            LpProblem modified = CloneProblem(RequireCanonical(currentResult).Original);
            double oldValue = modified.ObjectiveCoeffs[originalVariableIndex];
            modified.ObjectiveCoeffs[originalVariableIndex] = newCoefficient;
            return SolveChange(modified,
                $"Changed objective coefficient of x{originalVariableIndex + 1} from {F3(oldValue)} to {F3(newCoefficient)}.");
        }

        public ChangeResult ApplyRhsChange(
            PrimalSimplexSolver.Result currentResult,
            int originalConstraintIndex,
            double newRhs)
        {
            EnsureOptimal(currentResult);
            LpProblem modified = CloneProblem(RequireCanonical(currentResult).Original);
            double oldValue = modified.Rhs[originalConstraintIndex];
            modified.Rhs[originalConstraintIndex] = newRhs;
            return SolveChange(modified,
                $"Changed RHS of constraint {originalConstraintIndex + 1} from {F3(oldValue)} to {F3(newRhs)}.");
        }

        public ChangeResult ApplyNonBasicColumnCoefficientChange(
            PrimalSimplexSolver.Result currentResult,
            int originalConstraintIndex,
            int originalVariableIndex,
            double newCoefficient)
        {
            // Validate that the selected variable is currently a non-basic single column.
            GetNonBasicColumnCoefficientRange(currentResult, originalConstraintIndex, originalVariableIndex);

            LpProblem modified = CloneProblem(RequireCanonical(currentResult).Original);
            double oldValue = modified.ConstraintCoeffs[originalConstraintIndex][originalVariableIndex];
            modified.ConstraintCoeffs[originalConstraintIndex][originalVariableIndex] = newCoefficient;
            return SolveChange(modified,
                $"Changed a[{originalConstraintIndex + 1},{originalVariableIndex + 1}] from {F3(oldValue)} to {F3(newCoefficient)}.");
        }

        public ChangeResult AddNewActivity(
            PrimalSimplexSolver.Result currentResult,
            double objectiveCoefficient,
            IReadOnlyList<double> technologicalCoefficients,
            string signRestriction)
        {
            EnsureOptimal(currentResult);
            LpProblem modified = CloneProblem(RequireCanonical(currentResult).Original);

            if (technologicalCoefficients.Count != modified.NumConstraints)
                throw new ArgumentException("A new activity needs one technological coefficient for every original constraint.");

            string restriction = signRestriction.Trim().ToLowerInvariant();
            if (restriction is not ("+" or "-" or "urs" or "int" or "bin"))
                throw new FormatException("Sign restriction must be +, -, urs, int or bin.");

            modified.ObjectiveCoeffs.Add(objectiveCoefficient);
            modified.SignRestrictions.Add(restriction);
            for (int i = 0; i < modified.NumConstraints; i++)
                modified.ConstraintCoeffs[i].Add(technologicalCoefficients[i]);

            return SolveChange(modified,
                $"Added new activity x{modified.NumVariables} with objective coefficient {F3(objectiveCoefficient)}.");
        }

        public ChangeResult AddNewConstraint(
            PrimalSimplexSolver.Result currentResult,
            IReadOnlyList<double> coefficients,
            string relation,
            double rhs)
        {
            EnsureOptimal(currentResult);
            LpProblem modified = CloneProblem(RequireCanonical(currentResult).Original);

            if (coefficients.Count != modified.NumVariables)
                throw new ArgumentException("A new constraint needs one coefficient for every decision variable.");

            relation = relation.Trim();
            if (relation is not ("<=" or ">=" or "="))
                throw new FormatException("Constraint relation must be <=, >= or =.");

            modified.ConstraintCoeffs.Add(coefficients.ToList());
            modified.Relations.Add(relation);
            modified.Rhs.Add(rhs);

            return SolveChange(modified,
                $"Added new constraint {modified.NumConstraints}: {relation} {F3(rhs)}.");
        }

        public bool IsOriginalVariableBasic(PrimalSimplexSolver.Result result, int originalVariableIndex)
        {
            EnsureOptimal(result);
            CanonicalProblem canonical = RequireCanonical(result);
            return canonical.OriginalVariableColumns[originalVariableIndex]
                .Any(result.FinalBasis.Contains);
        }

        private RangeResult GetObjectiveCoefficientRangeCore(
            PrimalSimplexSolver.Result result,
            int originalVariableIndex,
            string label)
        {
            CanonicalProblem canonical = RequireCanonical(result);
            LpProblem lp = canonical.Original;
            int n = canonical.NumVarsTotal;
            var basisSet = new HashSet<int>(result.FinalBasis);
            var artificial = new HashSet<int>(canonical.ArtificialVarIndices);

            // dc/d(original coefficient) for every canonical column.
            var dCost = new double[n];
            double objectiveSense = lp.IsMaximization ? 1.0 : -1.0;
            for (int k = 0; k < canonical.OriginalVariableColumns[originalVariableIndex].Count; k++)
            {
                int col = canonical.OriginalVariableColumns[originalVariableIndex][k];
                double multiplier = canonical.OriginalVariableMultipliers[originalVariableIndex][k];
                dCost[col] = objectiveSense * multiplier;
            }

            double lowerDelta = double.NegativeInfinity;
            double upperDelta = double.PositiveInfinity;

            for (int col = 0; col < n; col++)
            {
                if (basisSet.Contains(col) || artificial.Contains(col))
                    continue;

                double slope = -dCost[col];
                for (int row = 0; row < result.FinalBasis.Count; row++)
                {
                    int basicCol = result.FinalBasis[row];
                    slope += dCost[basicCol] * result.FinalTableau[row + 1, col];
                }

                double reducedCost = result.FinalTableau[0, col];
                IntersectNonNegative(ref lowerDelta, ref upperDelta, reducedCost, slope);
            }

            double current = lp.ObjectiveCoeffs[originalVariableIndex];
            return new RangeResult
            {
                ItemName = $"{label} x{originalVariableIndex + 1} objective coefficient",
                CurrentValue = current,
                LowerBound = AddBound(current, lowerDelta),
                UpperBound = AddBound(current, upperDelta),
                Explanation = "Within this interval all non-basic reduced costs remain optimal for the current basis."
            };
        }

        private static ChangeResult SolveChange(LpProblem modified, string description)
        {
            CanonicalProblem canonical = CanonicalConverter.ToCanonicalForm(modified);
            var solver = new PrimalSimplexSolver();
            PrimalSimplexSolver.Result result = solver.Solve(canonical);
            return new ChangeResult
            {
                ModifiedProblem = modified,
                SolverResult = result,
                Description = description
            };
        }

        private static double[] GetEffectiveShadowPrices(PrimalSimplexSolver.Result result)
        {
            CanonicalProblem canonical = RequireCanonical(result);
            int m = canonical.NumConstraints;
            double[,] inverse = GetBasisInverse(result);
            var cB = new double[m];

            for (int i = 0; i < m; i++)
            {
                int col = result.FinalBasis[i];
                cB[i] = -canonical.TableauMatrix[0, col];
            }

            var y = new double[m];
            for (int j = 0; j < m; j++)
            {
                for (int i = 0; i < m; i++)
                    y[j] += cB[i] * inverse[i, j];
            }
            return y;
        }

        private static double[,] GetBasisInverse(PrimalSimplexSolver.Result result)
        {
            CanonicalProblem canonical = RequireCanonical(result);
            int m = canonical.NumConstraints;
            var basisMatrix = new double[m, m];

            for (int row = 0; row < m; row++)
            {
                for (int col = 0; col < m; col++)
                {
                    int variableColumn = result.FinalBasis[col];
                    basisMatrix[row, col] = canonical.TableauMatrix[row + 1, variableColumn];
                }
            }

            return Invert(basisMatrix);
        }

        private static double[,] Invert(double[,] matrix)
        {
            int n = matrix.GetLength(0);
            if (n != matrix.GetLength(1))
                throw new ArgumentException("Matrix must be square.");

            var augmented = new double[n, 2 * n];
            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < n; j++)
                    augmented[i, j] = matrix[i, j];
                augmented[i, n + i] = 1.0;
            }

            for (int pivot = 0; pivot < n; pivot++)
            {
                int bestRow = pivot;
                double best = Math.Abs(augmented[pivot, pivot]);
                for (int r = pivot + 1; r < n; r++)
                {
                    double value = Math.Abs(augmented[r, pivot]);
                    if (value > best)
                    {
                        best = value;
                        bestRow = r;
                    }
                }

                if (best <= Epsilon)
                    throw new InvalidOperationException("The final basis matrix is singular.");

                if (bestRow != pivot)
                {
                    for (int j = 0; j < 2 * n; j++)
                        (augmented[pivot, j], augmented[bestRow, j]) = (augmented[bestRow, j], augmented[pivot, j]);
                }

                double divisor = augmented[pivot, pivot];
                for (int j = 0; j < 2 * n; j++)
                    augmented[pivot, j] /= divisor;

                for (int r = 0; r < n; r++)
                {
                    if (r == pivot)
                        continue;
                    double factor = augmented[r, pivot];
                    if (Math.Abs(factor) <= Epsilon)
                        continue;
                    for (int j = 0; j < 2 * n; j++)
                        augmented[r, j] -= factor * augmented[pivot, j];
                }
            }

            var inverse = new double[n, n];
            for (int i = 0; i < n; i++)
                for (int j = 0; j < n; j++)
                    inverse[i, j] = augmented[i, n + j];
            return inverse;
        }

        private static void IntersectNonNegative(
            ref double lowerDelta,
            ref double upperDelta,
            double intercept,
            double slope)
        {
            if (intercept < -1e-7)
                throw new InvalidOperationException("The supplied basis is not optimal/dual-feasible.");

            if (Math.Abs(slope) <= Epsilon)
                return;

            double boundary = -intercept / slope;
            if (slope > 0.0)
                lowerDelta = Math.Max(lowerDelta, boundary);
            else
                upperDelta = Math.Min(upperDelta, boundary);
        }

        private static double AddBound(double current, double delta)
        {
            if (double.IsNegativeInfinity(delta)) return double.NegativeInfinity;
            if (double.IsPositiveInfinity(delta)) return double.PositiveInfinity;
            return current + delta;
        }

        private static void EnsureVariableStatus(
            PrimalSimplexSolver.Result result,
            int originalVariableIndex,
            bool mustBeBasic)
        {
            bool isBasic = result.CanonicalProblem!.OriginalVariableColumns[originalVariableIndex]
                .Any(result.FinalBasis.Contains);

            if (mustBeBasic && !isBasic)
                throw new InvalidOperationException($"x{originalVariableIndex + 1} is not a basic variable in the optimal tableau.");
            if (!mustBeBasic && isBasic)
                throw new InvalidOperationException($"x{originalVariableIndex + 1} is basic; select a non-basic variable.");
        }

        private static void EnsureVariableIndex(PrimalSimplexSolver.Result result, int index)
        {
            int count = RequireCanonical(result).Original.NumVariables;
            if (index < 0 || index >= count)
                throw new ArgumentOutOfRangeException(nameof(index));
        }

        private static void EnsureOptimal(PrimalSimplexSolver.Result result)
        {
            ArgumentNullException.ThrowIfNull(result);
            if (result.Status != PrimalSimplexSolver.SolutionStatus.Optimal)
                throw new InvalidOperationException("Sensitivity analysis requires an optimal simplex solution.");
            RequireCanonical(result);
            if (result.FinalBasis.Count == 0 || result.FinalTableau.Length == 0)
                throw new InvalidOperationException("The solver result does not contain final-basis information.");
        }

        private static CanonicalProblem RequireCanonical(PrimalSimplexSolver.Result result) =>
            result.CanonicalProblem ?? throw new InvalidOperationException("Canonical model information is unavailable.");

        public static LpProblem CloneProblem(LpProblem source)
        {
            return new LpProblem
            {
                IsMaximization = source.IsMaximization,
                ObjectiveCoeffs = new List<double>(source.ObjectiveCoeffs),
                ConstraintCoeffs = source.ConstraintCoeffs.Select(row => new List<double>(row)).ToList(),
                Relations = new List<string>(source.Relations),
                Rhs = new List<double>(source.Rhs),
                SignRestrictions = new List<string>(source.SignRestrictions)
            };
        }

        private static string FormatBound(double value)
        {
            if (double.IsNegativeInfinity(value)) return "-infinity";
            if (double.IsPositiveInfinity(value)) return "+infinity";
            return F3(value);
        }

        private static string F3(double value) => value.ToString("F3", CultureInfo.InvariantCulture);
    }
}
