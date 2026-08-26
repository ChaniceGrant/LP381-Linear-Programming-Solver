using System.Collections.Generic;

namespace LPR381Solver.Models
{
    public class LpProblem
    {
        public bool IsMaximization { get; set; }
        public List<double> ObjectiveCoeffs { get; set; } = new();
        public List<List<double>> ConstraintCoeffs { get; set; } = new();
        public List<string> Relations { get; set; } = new();
        public List<double> Rhs { get; set; } = new();
        public List<string> SignRestrictions { get; set; } = new();

        public int NumVariables => ObjectiveCoeffs.Count;
        public int NumConstraints => ConstraintCoeffs.Count;
    }

    public class CanonicalProblem
    {
        public LpProblem Original { get; set; } = new();
        public List<string> VariableNames { get; set; } = new();
        public double[,] TableauMatrix { get; set; } = new double[0, 0];
        public List<int> BasicVariables { get; set; } = new();
        public int NumVarsTotal { get; set; }
        public int NumConstraints { get; set; }
        public List<int> ArtificialVarIndices { get; set; } = new();

        // Maps each original variable back from its non-negative canonical columns.
        // Example: urs x1 = x1_pos - x1_neg.
        public List<List<int>> OriginalVariableColumns { get; set; } = new();
        public List<List<double>> OriginalVariableMultipliers { get; set; } = new();
    }
}
