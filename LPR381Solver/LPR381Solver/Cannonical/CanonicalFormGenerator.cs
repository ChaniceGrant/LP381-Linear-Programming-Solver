using System;
using System.Collections.Generic;
using LPR381Solver.Models;

namespace LPR381Solver.Services
{
    public static class CanonicalConverter
    {
        public static CanonicalProblem ToCanonicalForm(LpProblem lp)
        {
            if (lp == null)
                throw new ArgumentNullException(nameof(lp));

            var varNames = new List<string>();

            // Original decision variables
            for (int i = 1; i <= lp.NumVariables; i++)
            {
                varNames.Add($"x{i}");
            }

            int slackCount = 0;
            int surplusCount = 0;
            int artificialCount = 0;

            var additionalVariables = new List<string>();
            var basicVariables = new List<int>();
            var artificialIndices = new List<int>();

            /*
             * First determine which additional variables are required.
             *
             * <=  : add slack variable
             * >=  : add surplus variable and artificial variable
             * =   : add artificial variable
             */
            for (int i = 0; i < lp.NumConstraints; i++)
            {
                string relation = lp.Relations[i].Trim();

                if (relation == "<=")
                {
                    slackCount++;

                    additionalVariables.Add($"s{slackCount}");

                    int slackIndex =
                        lp.NumVariables + additionalVariables.Count - 1;

                    basicVariables.Add(slackIndex);
                }
                else if (relation == ">=")
                {
                    surplusCount++;
                    additionalVariables.Add($"e{surplusCount}");

                    artificialCount++;
                    additionalVariables.Add($"a{artificialCount}");

                    int artificialIndex =
                        lp.NumVariables + additionalVariables.Count - 1;

                    artificialIndices.Add(artificialIndex);

                    // Artificial variable is initially basic.
                    basicVariables.Add(artificialIndex);
                }
                else if (relation == "=")
                {
                    artificialCount++;
                    additionalVariables.Add($"a{artificialCount}");

                    int artificialIndex =
                        lp.NumVariables + additionalVariables.Count - 1;

                    artificialIndices.Add(artificialIndex);

                    // Artificial variable is initially basic.
                    basicVariables.Add(artificialIndex);
                }
                else
                {
                    throw new InvalidOperationException(
                        $"Invalid constraint relation '{relation}' at constraint {i + 1}."
                    );
                }
            }

            varNames.AddRange(additionalVariables);

            int totalVars = varNames.Count;

            /*
             * Tableau layout:
             *
             * Row 0                  = objective function
             * Rows 1..m              = constraints
             * Column totalVars       = RHS
             */
            double[,] matrix =
                new double[lp.NumConstraints + 1, totalVars + 1];

            /*
             * Objective row.
             *
             * We use the convention:
             *
             *     z - c1x1 - c2x2 - ... = 0
             *
             * Therefore a maximisation problem uses -c_j.
             *
             * For minimisation, the objective is represented as the
             * maximisation of the negative objective. The original
             * objective direction is retained in lp.IsMaximization so
             * the final result can be converted back by the solver.
             */
            for (int j = 0; j < lp.NumVariables; j++)
            {
                matrix[0, j] = -lp.ObjectiveCoeffs[j];
            }

            /*
             * Build the constraint rows.
             *
             * Before constructing the row, make sure the RHS is
             * non-negative. If the RHS is negative, multiply the entire
             * constraint by -1 and reverse the relation.
             */
            int additionalVariablePosition = 0;

            for (int i = 0; i < lp.NumConstraints; i++)
            {
                int row = i + 1;

                if (lp.ConstraintCoeffs[i].Count != lp.NumVariables)
                {
                    throw new InvalidOperationException(
                        $"Constraint {i + 1} has {lp.ConstraintCoeffs[i].Count} coefficients, " +
                        $"but {lp.NumVariables} decision variables were expected."
                    );
                }

                string relation = lp.Relations[i].Trim();
                double rhs = lp.Rhs[i];

                // Copy the original coefficients.
                for (int j = 0; j < lp.NumVariables; j++)
                {
                    matrix[row, j] = lp.ConstraintCoeffs[i][j];
                }

                /*
                 * If RHS is negative, multiply the entire constraint
                 * by -1 and reverse the relation.
                 */
                if (rhs < 0)
                {
                    for (int j = 0; j < lp.NumVariables; j++)
                    {
                        matrix[row, j] = -matrix[row, j];
                    }

                    rhs = -rhs;

                    relation = ReverseRelation(relation);
                }

                matrix[row, totalVars] = rhs;

                /*
                 * Add the appropriate canonical variable.
                 */
                if (relation == "<=")
                {
                    // + slack variable
                    matrix[row, lp.NumVariables + additionalVariablePosition] = 1.0;

                    additionalVariablePosition++;
                }
                else if (relation == ">=")
                {
                    // - surplus variable
                    matrix[row, lp.NumVariables + additionalVariablePosition] = -1.0;
                    additionalVariablePosition++;

                    // + artificial variable
                    matrix[row, lp.NumVariables + additionalVariablePosition] = 1.0;
                    additionalVariablePosition++;
                }
                else if (relation == "=")
                {
                    // + artificial variable
                    matrix[row, lp.NumVariables + additionalVariablePosition] = 1.0;
                    additionalVariablePosition++;
                }
                else
                {
                    throw new InvalidOperationException(
                        $"Invalid constraint relation '{relation}' at constraint {i + 1}."
                    );
                }
            }

            return new CanonicalProblem
            {
                Original = lp,
                VariableNames = varNames,
                TableauMatrix = matrix,
                BasicVariables = basicVariables,
                NumVarsTotal = totalVars,
                NumConstraints = lp.NumConstraints,
                ArtificialVarIndices = artificialIndices
            };
        }

        private static string ReverseRelation(string relation)
        {
            return relation switch
            {
                "<=" => ">=",
                ">=" => "<=",
                "=" => "=",
                _ => throw new InvalidOperationException(
                    $"Cannot reverse invalid relation '{relation}'."
                )
            };
        }
    }
}