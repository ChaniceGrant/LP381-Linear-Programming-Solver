using System;
using System.Collections.Generic;
using LPR381Solver.Models;

namespace LPR381Solver.Services
{
    public static class CanonicalConverter
    {
        private sealed class NormalizedConstraint
        {
            public List<double> Coefficients { get; init; } = new();
            public string Relation { get; init; } = string.Empty;
            public double Rhs { get; init; }
        }

        public static CanonicalProblem ToCanonicalForm(LpProblem lp)
        {
            ArgumentNullException.ThrowIfNull(lp);
            ValidateModel(lp);

            var variableNames = new List<string>();
            var originalColumns = new List<List<int>>();
            var originalMultipliers = new List<List<double>>();

            // Expand original variables into non-negative canonical variables.
            for (int j = 0; j < lp.NumVariables; j++)
            {
                string restriction = lp.SignRestrictions[j].ToLowerInvariant();
                var columns = new List<int>();
                var multipliers = new List<double>();

                if (restriction == "-")
                {
                    columns.Add(variableNames.Count);
                    multipliers.Add(-1.0);
                    variableNames.Add($"x{j + 1}_neg");
                }
                else if (restriction == "urs")
                {
                    columns.Add(variableNames.Count);
                    multipliers.Add(1.0);
                    variableNames.Add($"x{j + 1}_pos");

                    columns.Add(variableNames.Count);
                    multipliers.Add(-1.0);
                    variableNames.Add($"x{j + 1}_neg");
                }
                else
                {
                    // +, int and bin are non-negative in the LP relaxation.
                    columns.Add(variableNames.Count);
                    multipliers.Add(1.0);
                    variableNames.Add($"x{j + 1}");
                }

                originalColumns.Add(columns);
                originalMultipliers.Add(multipliers);
            }

            int structuralVariableCount = variableNames.Count;
            var constraints = new List<NormalizedConstraint>();

            // Transform original constraints to the expanded non-negative variables.
            for (int i = 0; i < lp.NumConstraints; i++)
            {
                var expanded = new double[structuralVariableCount];

                for (int originalVar = 0; originalVar < lp.NumVariables; originalVar++)
                {
                    double a = lp.ConstraintCoeffs[i][originalVar];
                    for (int k = 0; k < originalColumns[originalVar].Count; k++)
                    {
                        int col = originalColumns[originalVar][k];
                        expanded[col] += a * originalMultipliers[originalVar][k];
                    }
                }

                AddNormalizedConstraint(
                    constraints,
                    new List<double>(expanded),
                    lp.Relations[i],
                    lp.Rhs[i]);
            }

            // A binary variable's LP relaxation includes x <= 1.
            for (int j = 0; j < lp.NumVariables; j++)
            {
                if (!lp.SignRestrictions[j].Equals("bin", StringComparison.OrdinalIgnoreCase))
                    continue;

                var bound = new double[structuralVariableCount];
                bound[originalColumns[j][0]] = 1.0;
                AddNormalizedConstraint(constraints, new List<double>(bound), "<=", 1.0);
            }

            int slackCount = 0;
            int surplusCount = 0;
            int artificialCount = 0;
            var basicVariables = new List<int>();
            var artificialIndices = new List<int>();

            // Allocate auxiliary columns AFTER RHS normalization so the relation is final.
            foreach (NormalizedConstraint constraint in constraints)
            {
                switch (constraint.Relation)
                {
                    case "<=":
                        variableNames.Add($"s{++slackCount}");
                        basicVariables.Add(variableNames.Count - 1);
                        break;

                    case ">=":
                        variableNames.Add($"e{++surplusCount}");
                        variableNames.Add($"a{++artificialCount}");
                        artificialIndices.Add(variableNames.Count - 1);
                        basicVariables.Add(variableNames.Count - 1);
                        break;

                    case "=":
                        variableNames.Add($"a{++artificialCount}");
                        artificialIndices.Add(variableNames.Count - 1);
                        basicVariables.Add(variableNames.Count - 1);
                        break;
                }
            }

            int totalVariables = variableNames.Count;
            int m = constraints.Count;
            var matrix = new double[m + 1, totalVariables + 1];

            // Convert min c*x to max (-c*x), then use z - c_eff*x = 0.
            for (int originalVar = 0; originalVar < lp.NumVariables; originalVar++)
            {
                double effectiveC = lp.IsMaximization
                    ? lp.ObjectiveCoeffs[originalVar]
                    : -lp.ObjectiveCoeffs[originalVar];

                for (int k = 0; k < originalColumns[originalVar].Count; k++)
                {
                    int col = originalColumns[originalVar][k];
                    matrix[0, col] = -effectiveC * originalMultipliers[originalVar][k];
                }
            }

            int auxiliaryColumn = structuralVariableCount;
            for (int i = 0; i < m; i++)
            {
                int row = i + 1;
                NormalizedConstraint constraint = constraints[i];

                for (int j = 0; j < structuralVariableCount; j++)
                    matrix[row, j] = constraint.Coefficients[j];

                matrix[row, totalVariables] = constraint.Rhs;

                switch (constraint.Relation)
                {
                    case "<=":
                        matrix[row, auxiliaryColumn++] = 1.0;
                        break;
                    case ">=":
                        matrix[row, auxiliaryColumn++] = -1.0;
                        matrix[row, auxiliaryColumn++] = 1.0;
                        break;
                    case "=":
                        matrix[row, auxiliaryColumn++] = 1.0;
                        break;
                }
            }

            return new CanonicalProblem
            {
                Original = lp,
                VariableNames = variableNames,
                TableauMatrix = matrix,
                BasicVariables = basicVariables,
                NumVarsTotal = totalVariables,
                NumConstraints = m,
                ArtificialVarIndices = artificialIndices,
                OriginalVariableColumns = originalColumns,
                OriginalVariableMultipliers = originalMultipliers
            };
        }

        private static void AddNormalizedConstraint(
            List<NormalizedConstraint> destination,
            List<double> coefficients,
            string relation,
            double rhs)
        {
            relation = relation.Trim();
            if (relation != "<=" && relation != ">=" && relation != "=")
                throw new FormatException($"Invalid constraint relation '{relation}'.");

            if (rhs < 0.0)
            {
                for (int j = 0; j < coefficients.Count; j++)
                    coefficients[j] = -coefficients[j];

                rhs = -rhs;
                relation = relation switch
                {
                    "<=" => ">=",
                    ">=" => "<=",
                    _ => "="
                };
            }

            destination.Add(new NormalizedConstraint
            {
                Coefficients = coefficients,
                Relation = relation,
                Rhs = rhs
            });
        }

        private static void ValidateModel(LpProblem lp)
        {
            if (lp.NumVariables == 0)
                throw new FormatException("The model must contain at least one decision variable.");
            if (lp.NumConstraints == 0)
                throw new FormatException("The model must contain at least one constraint.");
            if (lp.SignRestrictions.Count != lp.NumVariables)
                throw new FormatException("The number of sign restrictions must match the number of variables.");
            if (lp.Relations.Count != lp.NumConstraints || lp.Rhs.Count != lp.NumConstraints)
                throw new FormatException("Constraint relations/RHS counts do not match the number of constraints.");

            for (int i = 0; i < lp.NumConstraints; i++)
                if (lp.ConstraintCoeffs[i].Count != lp.NumVariables)
                    throw new FormatException($"Constraint {i + 1} does not contain {lp.NumVariables} coefficients.");
        }
    }
}
