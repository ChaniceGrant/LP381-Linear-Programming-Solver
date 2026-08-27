using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using LPR381Solver.Models;
using System.Globalization; 

namespace LPR381Solver.Algorithms
{
    public class SimplexBranchNode
    {
        public string Label { get; set; }
        public List<Constraint> ExtraConstraints { get; set; }
        public double? Bound  {get; set;}
        public string Status { get; set; }

        public SimplexBranchNode(string label, List<Constraint> extraConstraints)
        {
            Label = label;
            ExtraConstraints = new List<Constraint>(extraConstraints);
            Status = "PENDING";
        }
    }

    public class BranchAndBoundSimplex
    {
        private const double Epsilon = 1e-6;
        private const double IntegerEpsilon = 1e-4;

        private const int MaxNodes = 10000;

        private LPModel _rootModel;
        private List<int> _integerVarIndices;
        private bool _isMax;
        private double _bestObjective;
        
        private Dictionary<string, double> _bestSolution;

        private readonly List<SimplexBranchNode> _nodes = new List<SimplexBranchNode>();
        private readonly StringBuilder _log = new StringBuilder();

        public IReadOnlyList<SimplexBranchNode> Nodes => _nodes;
        public double BestObjective => _bestObjective;
        public Dictionary<string, double> BestSolution => _bestSolution;
        public string Status { get; private set; }
        public string Log => _log.ToString();

        public void Solve(LpProblem problem)
        {
            LPModel model = ConvertToLPModel(problem);
            Solve(model);
        }

        private static LPModel ConvertToLPModel(LpProblem lp)
        {
            LPModel model = new LPModel();
            model.ObjectiveType = lp.IsMaximization ? "max" : "min";

            for(int i = 0; i < lp.ObjectiveCoeffs.Count; i ++)
            {
                string name = "x" + (i+1);
                string sign = (lp.SignRestrictions != null && i < lp.SignRestrictions.Count)
                    ? lp.SignRestrictions[i] : "+";
                model.Variables.Add(new Variable(name, lp.ObjectiveCoeffs[i], sign));
            }

            for (int i = 0; i < lp.ConstraintCoeffs.Count; i++)
            {
                model.Constraints.Add(new Constraint(
                    new List<double>(lp.ConstraintCoeffs[i]),  lp.Relations[i], lp.Rhs[i]));
            }
            return model;
        }

        public void Solve(LPModel model)
        {
            Validate(model);
            _rootModel = model;
            _isMax = string.Equals(model.ObjectiveType,  "max", StringComparison.OrdinalIgnoreCase);

            _integerVarIndices = new List<int>();
            for(int i = 0; i < model.Variables.Count; i++)
            {
                string sr = (model.Variables[i].SignRestriction ??  "+").Trim().ToLowerInvariant();
                if(sr == "int" || sr == "bin") _integerVarIndices.Add(i);
            }

            WriteHeader();

            if(_integerVarIndices.Count == 0)
            {
                _log.AppendLine("No variables are restricted to int/bin - there is nothing to branch on");
                _log.AppendLine("Solving as a plain LP relaxation instead");
                _log.AppendLine();
                SolveAsPlainLP();
                return;
            }

            _bestObjective = _isMax ? double.NegativeInfinity : double.PositiveInfinity;
            _bestSolution = null;

            Stack<SimplexBranchNode> stack = new Stack<SimplexBranchNode>();
            stack.Push(new SimplexBranchNode("1", new List<Constraint>()));

            int nodeCount = 0;

            while (stack.Count > 0)
            {
                if(++nodeCount > MaxNodes)
                {
                    _log.AppendLine("Node limit (" + MaxNodes + ") reached - stopping early");
                    _log.AppendLine();
                    break;
                }

                SimplexBranchNode node = stack.Pop();
                _nodes.Add(node);
                ProcessNode(node, stack);
            }

            Status = _bestSolution != null ? "Optimal" : "Feasible solution found";
            WriteBestCandidate();
        }

        private void Validate(LPModel model)
        {
            if (model == null) throw new ArgumentNullException(nameof(model));
            if (model.Variables == null || model.Variables.Count == 0)
                throw new InvalidOperationException("There are no decision variables for this model");
            if (model.Constraints == null)
                throw new InvalidOperationException("There are no constraints");
        }

        private void ProcessNode(SimplexBranchNode node, Stack<SimplexBranchNode> stack)
        {
            _log.AppendLine("------------------------------------------------");
            _log.AppendLine("Sub problem: " + node.Label);
            _log.AppendLine("------------------------------------------------");

            if(node.ExtraConstraints.Count == 0)
            {
                _log.AppendLine("Bounds applied: none (root LP relaxation)");
            }
            else
            {
                _log.AppendLine("Bounds applies: ");
                foreach (Constraint c in node.ExtraConstraints)
                    _log.AppendLine(" " + DescribeBoundConstraints(c));
            }

            _log.AppendLine();

            LPModel nodeModel =  BuildNodeModel(node);

            RevisedPrimalSimplex solver = new RevisedPrimalSimplex();
            solver.Solve(nodeModel);

            _log.AppendLine("--- Revised Primal Simplex trace for sub-problem " + node.Label + "---");
            _log.AppendLine(solver.Log);

            if (solver.Status == "Infeasible")
            {
                node.Status = "FATHOMED - INFEASIBLE";
                WriteNodeResult(node);
                return;
            }

            if (solver.Status == "Unbounded")
            {
                node.Status = "FATHOMES - RELAXATION IS UNBOUNDED";
                WriteNodeResult(node);
                return;
            }

            double bound = solver.ObjectiveValue;
            node.Bound = bound;

            if(_bestSolution != null && !IsBoundBetterThanIncumbent(bound))
            {
                node.Status = string.Format(CultureInfo.InvariantCulture,
                "FATHOMD - bound {0:F3} is not better than incumbent {1:F3}",
                bound, _bestObjective);
                WriteNodeResult(node);
                return;
            }

            int fractionalIndex = -1;
            double fractionalValue = 0.0;
            foreach (int idx in _integerVarIndices)
            {
                double value = solver.VariableValues[_rootModel.Variables[idx].Name];
                double rounded = Math.Round(value, MidpointRounding.AwayFromZero);
                if(Math.Abs(value - rounded) > IntegerEpsilon)
                {
                    fractionalIndex = idx;
                    fractionalValue = value;
                    break;
                }
            }

            if (fractionalIndex == -1)
            {
                //if every int or bin variable landed on an integer value then this is an candidate solution
                node.Status = "CANDIDATE - relaxtation is integer-feasible";
                WriteNodeResult(node);

                if(_bestSolution == null || IsBoundBetterThanIncumbent(bound))
                {
                    _bestObjective = bound;
                    _bestSolution = new Dictionary<string, double>(solver.VariableValues);
                    _log.AppendLine(string.Format(CultureInfo.InvariantCulture,
                    "  -> New best candidate. z = {0:F3}", _bestObjective));
                    _log.AppendLine();
                }
                return;
            }

            string branchVarName = _rootModel.Variables[fractionalIndex].Name;
            double floorVal = Math.Floor(fractionalValue);
            double ceilVal = Math.Ceiling(fractionalValue);

            node.Status = string.Format(CultureInfo.InvariantCulture,
            "BRANCH on {0} (fractional value {1:F3})", branchVarName, fractionalValue);
            WriteNodeResult(node);

            List<double> unitCoeffs = BuildUnitCoefficients(fractionalIndex);

            List<Constraint> floorExtras = new List<Constraint>(node.ExtraConstraints)
            {
                new Constraint(unitCoeffs, "<=", floorVal)
            };
            List<Constraint> ceilExtras =  new List<Constraint>(node.ExtraConstraints)
            {
                new Constraint(new List<double>(unitCoeffs), ">=", ceilVal)
            };

            stack.Push(new SimplexBranchNode(node.Label + ".2", ceilExtras));
			stack.Push(new SimplexBranchNode(node.Label + ".1", floorExtras));
        }

        private bool IsBoundBetterThanIncumbent(double bound)
		{
			return _isMax
				? bound > _bestObjective + Epsilon
				: bound < _bestObjective - Epsilon;
		}
 
		private LPModel BuildNodeModel(SimplexBranchNode node)
		{
			LPModel nodeModel = new LPModel();
			nodeModel.ObjectiveType = _rootModel.ObjectiveType;
			nodeModel.Variables = _rootModel.Variables; // not mutated anywhere - safe to share
			nodeModel.Constraints = new List<Constraint>(_rootModel.Constraints);
			nodeModel.Constraints.AddRange(node.ExtraConstraints);
			return nodeModel;
		}
 
		private List<double> BuildUnitCoefficients(int variableIndex)
		{
			List<double> coeffs = new List<double>(new double[_rootModel.Variables.Count]);
			coeffs[variableIndex] = 1.0;
			return coeffs;
		}
 
		private string DescribeBoundConstraints(Constraint c)
		{
			int idx = c.Coefficients.FindIndex(v => Math.Abs(v) > Epsilon);
			string varName = idx >= 0 ? _rootModel.Variables[idx].Name : "?";
			return varName + " " + c.Relation + " " + c.RightHandSide.ToString("F3", CultureInfo.InvariantCulture);
		}
 
		private void SolveAsPlainLP()
		{
			RevisedPrimalSimplex solver = new RevisedPrimalSimplex();
			solver.Solve(_rootModel);
			_log.AppendLine(solver.Log);
 
			Status = solver.Status;
			if (solver.Status == "Optimal")
			{
				_bestObjective = solver.ObjectiveValue;
				_bestSolution = new Dictionary<string, double>(solver.VariableValues);
			}
		}
 
		private void WriteHeader()
		{
			_log.AppendLine("==========================================================");
			_log.AppendLine("   BRANCH AND BOUND SIMPLEX ALGORITHM");
			_log.AppendLine("==========================================================");
			_log.AppendLine();
			_log.AppendLine("Objective: " + _rootModel.ObjectiveType);
			_log.AppendLine("Integer/binary restricted variables: " +
				(_integerVarIndices.Count == 0
					? "none"
					: string.Join(", ", _integerVarIndices.Select(i => _rootModel.Variables[i].Name))));
			_log.AppendLine();
		}
 
		private void WriteNodeResult(SimplexBranchNode node)
		{
			_log.AppendLine("Result: " + node.Status);
			_log.AppendLine();
		}
 
		private void WriteBestCandidate()
		{
			_log.AppendLine("==========================================================");
			_log.AppendLine("   BEST CANDIDATE");
			_log.AppendLine("==========================================================");
 
			if (_bestSolution == null)
			{
				_log.AppendLine("  No integer-feasible solution was found.");
				return;
			}
 
			foreach (KeyValuePair<string, double> kv in _bestSolution)
				_log.AppendLine(string.Format(CultureInfo.InvariantCulture, "  {0} = {1:F3}", kv.Key, kv.Value));
 
			_log.AppendLine(string.Format(CultureInfo.InvariantCulture,
				"  Optimal objective value z = {0:F3}", _bestObjective));
			_log.AppendLine(string.Format(CultureInfo.InvariantCulture,
				"  Sub-problems explored = {0}", _nodes.Count));
		}
    }

}
