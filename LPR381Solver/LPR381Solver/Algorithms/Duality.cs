using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using LPR381Solver.Models;

namespace LPR381Solver.Services
{
    public sealed class DualityService
    {
        private const double Tolerance = 1e-6;

        public sealed class DualBuildResult
        {
            public LpProblem RelaxedPrimal { get; init; } = new();
            public LpProblem Dual { get; init; } = new();
            public string Description { get; init; } = string.Empty;
        }

        public sealed class DualSolveResult
        {
            public DualBuildResult Build { get; init; } = new();
            public PrimalSimplexSolver.Result Result { get; init; } = new();
        }

        public sealed class VerificationResult
        {
            public PrimalSimplexSolver.Result PrimalResult { get; init; } = new();
            public PrimalSimplexSolver.Result DualResult { get; init; } = new();
            public bool WeakDualitySatisfied { get; init; }
            public bool StrongDualitySatisfied { get; init; }
            public string Message { get; init; } = string.Empty;
        }

        /// <summary>
        /// Builds the mathematical dual. Integer restrictions are relaxed. Binary
        /// variables are relaxed to 0 <= x <= 1, so their upper bounds become
        /// explicit primal constraints before the dual is constructed.
        /// </summary>
        public DualBuildResult BuildDual(LpProblem original)
        {
            ArgumentNullException.ThrowIfNull(original);
            LpProblem primal = BuildContinuousRelaxation(original);
            var dual = new LpProblem
            {
                IsMaximization = !primal.IsMaximization
            };

            // One dual variable for every primal constraint.
            dual.ObjectiveCoeffs.AddRange(primal.Rhs);
            for (int i = 0; i < primal.NumConstraints; i++)
                dual.SignRestrictions.Add(DualVariableRestriction(primal.IsMaximization, primal.Relations[i]));

            // One dual constraint for every primal decision variable.
            for (int j = 0; j < primal.NumVariables; j++)
            {
                var row = new List<double>();
                for (int i = 0; i < primal.NumConstraints; i++)
                    row.Add(primal.ConstraintCoeffs[i][j]);

                dual.ConstraintCoeffs.Add(row);
                dual.Relations.Add(DualConstraintRelation(
                    primal.IsMaximization,
                    primal.SignRestrictions[j]));
                dual.Rhs.Add(primal.ObjectiveCoeffs[j]);
            }

            return new DualBuildResult
            {
                RelaxedPrimal = primal,
                Dual = dual,
                Description = FormatDual(primal, dual)
            };
        }

        public DualSolveResult SolveDual(LpProblem original)
        {
            DualBuildResult build = BuildDual(original);
            CanonicalProblem canonical = CanonicalConverter.ToCanonicalForm(build.Dual);
            var solver = new PrimalSimplexSolver();
            return new DualSolveResult
            {
                Build = build,
                Result = solver.Solve(canonical)
            };
        }

        public VerificationResult VerifyDuality(LpProblem original)
        {
            DualBuildResult build = BuildDual(original);
            var solver = new PrimalSimplexSolver();

            PrimalSimplexSolver.Result primalResult = solver.Solve(
                CanonicalConverter.ToCanonicalForm(build.RelaxedPrimal));
            PrimalSimplexSolver.Result dualResult = solver.Solve(
                CanonicalConverter.ToCanonicalForm(build.Dual));

            if (primalResult.Status != PrimalSimplexSolver.SolutionStatus.Optimal ||
                dualResult.Status != PrimalSimplexSolver.SolutionStatus.Optimal)
            {
                return new VerificationResult
                {
                    PrimalResult = primalResult,
                    DualResult = dualResult,
                    Message = $"Duality could not be numerically verified because primal status is {primalResult.Status} " +
                              $"and dual status is {dualResult.Status}."
                };
            }

            double p = primalResult.ObjectiveValue;
            double d = dualResult.ObjectiveValue;
            bool weak = build.RelaxedPrimal.IsMaximization
                ? p <= d + Tolerance
                : p + Tolerance >= d;
            bool strong = Math.Abs(p - d) <= Tolerance;

            string message = strong
                ? $"STRONG DUALITY VERIFIED: primal objective = {F3(p)} and dual objective = {F3(d)}. " +
                  "Weak duality is also satisfied."
                : weak
                    ? $"WEAK DUALITY SATISFIED: primal objective = {F3(p)}, dual objective = {F3(d)}, " +
                      "but equality was not reached within tolerance."
                    : $"DUALITY CHECK FAILED numerically: primal objective = {F3(p)}, dual objective = {F3(d)}.";

            return new VerificationResult
            {
                PrimalResult = primalResult,
                DualResult = dualResult,
                WeakDualitySatisfied = weak,
                StrongDualitySatisfied = strong,
                Message = message
            };
        }

        private static LpProblem BuildContinuousRelaxation(LpProblem source)
        {
            LpProblem relaxed = SensitivityAnalysisService.CloneProblem(source);
            int originalVariableCount = relaxed.NumVariables;

            // int -> non-negative continuous. bin -> non-negative continuous + x <= 1.
            for (int j = 0; j < originalVariableCount; j++)
            {
                string restriction = relaxed.SignRestrictions[j].ToLowerInvariant();
                if (restriction == "int")
                {
                    relaxed.SignRestrictions[j] = "+";
                }
                else if (restriction == "bin")
                {
                    relaxed.SignRestrictions[j] = "+";
                    var row = Enumerable.Repeat(0.0, originalVariableCount).ToList();
                    row[j] = 1.0;
                    relaxed.ConstraintCoeffs.Add(row);
                    relaxed.Relations.Add("<=");
                    relaxed.Rhs.Add(1.0);
                }
            }

            return relaxed;
        }

        private static string DualVariableRestriction(bool primalIsMax, string primalRelation)
        {
            return primalRelation.Trim() switch
            {
                "=" => "urs",
                "<=" => primalIsMax ? "+" : "-",
                ">=" => primalIsMax ? "-" : "+",
                _ => throw new FormatException($"Invalid primal relation '{primalRelation}'.")
            };
        }

        private static string DualConstraintRelation(bool primalIsMax, string primalVariableRestriction)
        {
            string restriction = primalVariableRestriction.Trim().ToLowerInvariant();
            if (restriction is "int" or "bin") restriction = "+";

            if (restriction == "urs") return "=";
            if (primalIsMax)
                return restriction == "-" ? "<=" : ">=";
            return restriction == "-" ? ">=" : "<=";
        }

        private static string FormatDual(LpProblem primal, LpProblem dual)
        {
            var sb = new StringBuilder();
            sb.AppendLine("=== DUAL PROGRAMMING MODEL ===");
            sb.Append(dual.IsMaximization ? "max w = " : "min w = ");
            sb.AppendLine(FormatExpression(dual.ObjectiveCoeffs, "y"));
            sb.AppendLine("Subject to:");

            for (int i = 0; i < dual.NumConstraints; i++)
            {
                sb.Append("  ");
                sb.Append(FormatExpression(dual.ConstraintCoeffs[i], "y"));
                sb.AppendLine($" {dual.Relations[i]} {F3(dual.Rhs[i])}");
            }

            sb.AppendLine("Dual variable restrictions:");
            for (int j = 0; j < dual.NumVariables; j++)
                sb.AppendLine($"  y{j + 1}: {dual.SignRestrictions[j]}");

            if (primal.NumConstraints > 0)
                sb.AppendLine("(Integer restrictions were relaxed; binary upper bounds are represented explicitly when applicable.)");
            return sb.ToString();
        }

        private static string FormatExpression(IReadOnlyList<double> values, string variablePrefix)
        {
            var parts = new List<string>();
            for (int i = 0; i < values.Count; i++)
            {
                double value = values[i];
                string term = $"{F3(Math.Abs(value))}*{variablePrefix}{i + 1}";
                if (i == 0)
                    parts.Add(value < 0 ? "- " + term : term);
                else
                    parts.Add((value < 0 ? " - " : " + ") + term);
            }
            return string.Concat(parts);
        }

        private static string F3(double value) => value.ToString("F3", CultureInfo.InvariantCulture);
    }
}
