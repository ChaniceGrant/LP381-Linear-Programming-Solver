using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;


namespace LPR381Solver.Models
{
    public class LpProblem
    {
        public bool IsMaximization { get; set; }
        public List<double> ObjectiveCoeffs { get; set; } = new List<double>();
        public List<List<double>> ConstraintCoeffs { get; set; } = new List<List<double>>();
        public List<string> Relations { get; set; } = new List<string>();
        public List<double> Rhs { get; set; } = new List<double>();
        public List<string> SignRestrictions { get; set; } = new List<string>();

        public int NumVariables => ObjectiveCoeffs.Count;
        public int NumConstraints => ConstraintCoeffs.Count;
    }

    public class CanonicalProblem
    {
        public LpProblem Original { get; set; }
        public List<string> VariableNames { get; set; } = new List<string>();
        public double[,] TableauMatrix { get; set; } // Row 0 = Obj, Rows 1..M = Constraints
        public List<int> BasicVariables { get; set; } = new List<int>(); // Column index for each row basis
        public int NumVarsTotal { get; set; }
        public int NumConstraints { get; set; }
        public List<int> ArtificialVarIndices { get; set; } = new List<int>();
    }
}
