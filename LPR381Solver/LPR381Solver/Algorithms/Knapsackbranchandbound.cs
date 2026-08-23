using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;

using LPR381Solver.Models;

namespace LPR381Solver.Algorithms
{
	public class BranchNode
	{
		public string Label { get; set; }
		public Dictionary<int, int> FixedVariables { get; set; }
		public double? Bound { get; set; }
		public string Status { get; set; }

		public BranchNode(string label, Dictionary<int, int> fixedVariables)
		{
			Label = label;
			FixedVariables = new Dictionary<int, int>(fixedVariables);
			Status = "PENDING";
		}
	}

	public class KnapsackBranchAndBound
	{
		private const double Epsilon = 1e-9;
		private double[] _values;      
		private double[] _weights;      
		private double _capacity;          
		private int _variableCount;
		private string[] _variableNames;
		private int[] _rankOrder;         
		private double _bestValue;         
		private int[] _bestSolution;      
		private readonly List<BranchNode> _nodes = new List<BranchNode>();
		private readonly StringBuilder _log = new StringBuilder();
		public IReadOnlyList<BranchNode> Nodes => _nodes;
		public double BestValue => _bestValue;
		public int[] BestSolution => _bestSolution;
		public string Log => _log.ToString();

		public void Solve(LPModel model)
		{
			Validate(model);
			Extract(model);
			RankVariables();

			_bestValue = double.NegativeInfinity;
			_bestSolution = null;

			WriteHeader();
			WriteRankingTable();

			Stack<BranchNode> stack = new Stack<BranchNode>();
			stack.Push(new BranchNode("0", new Dictionary<int, int>()));

			while (stack.Count > 0)
			{
				BranchNode node = stack.Pop();
				_nodes.Add(node);

				int fractionalIndex;
				double fraction;
				double? bound = ComputeBound(node.FixedVariables, out fractionalIndex, out fraction);

				if (!bound.HasValue)
				{
					node.Bound = null;
					node.Status = "INFEASIBLE - total weight exceeds capacity";
					WriteNode(node, -1, 0);
					continue;
				}

				node.Bound = bound.Value;

				if (bound.Value <= _bestValue + Epsilon)
				{
					node.Status = string.Format(
						CultureInfo.InvariantCulture,
						"FATHOMED - bound {0:F3} is not better than best candidate {1:F3}",
						bound.Value, _bestValue);
					WriteNode(node, -1, 0);
					continue;
				}

				if (fractionalIndex < 0)
				{
					node.Status = "CANDIDATE - relaxation is all integer";
					int[] solution = BuildSolution(node.FixedVariables);
					WriteNode(node, -1, 0);

					if (bound.Value > _bestValue + Epsilon)
					{
						_bestValue = bound.Value;
						_bestSolution = solution;
						_log.AppendLine(string.Format(
							CultureInfo.InvariantCulture,
							"    -> New best candidate. z = {0:F3}", _bestValue));
						_log.AppendLine();
					}
					continue;
				}

				node.Status = string.Format(
					CultureInfo.InvariantCulture,
					"BRANCH on {0} (fractional value {1:F3})",
					_variableNames[fractionalIndex], fraction);
				WriteNode(node, fractionalIndex, fraction);

				Dictionary<int, int> zeroBranch = new Dictionary<int, int>(node.FixedVariables);
				zeroBranch[fractionalIndex] = 0;
				stack.Push(new BranchNode(node.Label + ".2", zeroBranch));

				Dictionary<int, int> oneBranch = new Dictionary<int, int>(node.FixedVariables);
				oneBranch[fractionalIndex] = 1;
				stack.Push(new BranchNode(node.Label + ".1", oneBranch));
			}

			WriteBestCandidate();
		}

		private void Validate(LPModel model)
		{
			if (model == null)
				throw new ArgumentNullException(nameof(model));

			if (model.Variables == null || model.Variables.Count == 0)
				throw new InvalidOperationException("The model has no decision variables.");

			if (!string.Equals(model.ObjectiveType, "max", StringComparison.OrdinalIgnoreCase))
				throw new InvalidOperationException(
					"Branch and Bound Knapsack requires a maximisation problem.");

			if (model.Constraints == null || model.Constraints.Count != 1)
				throw new InvalidOperationException(
					"Branch and Bound Knapsack requires exactly one constraint.");

			Constraint constraint = model.Constraints[0];

			if (constraint.Relation != "<=")
				throw new InvalidOperationException(
					"Branch and Bound Knapsack requires the constraint relation to be <=.");

			if (constraint.Coefficients.Count != model.Variables.Count)
				throw new InvalidOperationException(
					"The constraint has a different number of coefficients than there are variables.");

			foreach (Variable variable in model.Variables)
			{
				if (!string.Equals(variable.SignRestriction, "bin", StringComparison.OrdinalIgnoreCase))
					throw new InvalidOperationException(
						"Branch and Bound Knapsack requires every variable to be binary (bin).");
			}

			foreach (double weight in constraint.Coefficients)
			{
				if (weight <= 0)
					throw new InvalidOperationException(
						"Branch and Bound Knapsack requires strictly positive weights.");
			}
		}

		private void Extract(LPModel model)
		{
			_variableCount = model.Variables.Count;
			_values = new double[_variableCount];
			_weights = new double[_variableCount];
			_variableNames = new string[_variableCount];

			Constraint constraint = model.Constraints[0];

			for (int i = 0; i < _variableCount; i++)
			{
				_values[i] = model.Variables[i].ObjectiveCoefficient;
				_weights[i] = constraint.Coefficients[i];
				_variableNames[i] = model.Variables[i].Name;
			}

			_capacity = constraint.RightHandSide;
		}

		private void RankVariables()
		{
			_rankOrder = Enumerable.Range(0, _variableCount)
								   .OrderByDescending(i => _values[i] / _weights[i])
								   .ThenBy(i => i)  
								   .ToArray();
		}

		private double? ComputeBound(
			Dictionary<int, int> fixedVariables,
			out int fractionalIndex,
			out double fraction)
		{
			fractionalIndex = -1;
			fraction = 0.0;

			double value = 0.0;
			double remaining = _capacity;

			foreach (KeyValuePair<int, int> pair in fixedVariables)
			{
				if (pair.Value == 1)
				{
					value += _values[pair.Key];
					remaining -= _weights[pair.Key];
				}
			}

			if (remaining < -Epsilon)
				return null;   

			foreach (int i in _rankOrder)
			{
				if (fixedVariables.ContainsKey(i))
					continue;

				if (_weights[i] <= remaining + Epsilon)
				{
					remaining -= _weights[i];
					value += _values[i];
				}
				else
				{
					if (remaining > Epsilon)
					{
						fraction = remaining / _weights[i];
						value += _values[i] * fraction;
						fractionalIndex = i;
					}
					break;
				}
			}

			return value;
		}

		private int[] BuildSolution(Dictionary<int, int> fixedVariables)
		{
			int[] solution = new int[_variableCount];
			double remaining = _capacity;

			foreach (KeyValuePair<int, int> pair in fixedVariables)
			{
				solution[pair.Key] = pair.Value;
				if (pair.Value == 1)
					remaining -= _weights[pair.Key];
			}

			foreach (int i in _rankOrder)
			{
				if (fixedVariables.ContainsKey(i))
					continue;

				if (_weights[i] <= remaining + Epsilon)
				{
					solution[i] = 1;
					remaining -= _weights[i];
				}
			}

			return solution;
		}

		private void WriteHeader()
		{
			_log.AppendLine("==========================================================");
			_log.AppendLine("   BRANCH AND BOUND KNAPSACK ALGORITHM");
			_log.AppendLine("==========================================================");
			_log.AppendLine();

			_log.Append("Objective:  max z =");
			for (int i = 0; i < _variableCount; i++)
			{
				_log.Append(string.Format(CultureInfo.InvariantCulture,
					" {0}{1:F3}{2}", _values[i] >= 0 ? "+" : "-",
					Math.Abs(_values[i]), _variableNames[i]));
			}
			_log.AppendLine();

			_log.Append("Constraint: ");
			for (int i = 0; i < _variableCount; i++)
			{
				_log.Append(string.Format(CultureInfo.InvariantCulture,
					"{0}{1:F3}{2} ", _weights[i] >= 0 ? "+" : "-",
					Math.Abs(_weights[i]), _variableNames[i]));
			}
			_log.AppendLine(string.Format(CultureInfo.InvariantCulture,
				"<= {0:F3}", _capacity));
			_log.AppendLine();
		}

		private void WriteRankingTable()
		{
			_log.AppendLine("Step 1: Rank variables by value / weight ratio");
			_log.AppendLine("----------------------------------------------------------");
			_log.AppendLine("  Rank  Variable   Value    Weight   Ratio");

			for (int r = 0; r < _rankOrder.Length; r++)
			{
				int i = _rankOrder[r];
				_log.AppendLine(string.Format(CultureInfo.InvariantCulture,
					"  {0,-5} {1,-10} {2,7:F3} {3,8:F3} {4,8:F3}",
					r + 1, _variableNames[i], _values[i], _weights[i],
					_values[i] / _weights[i]));
			}

			_log.AppendLine();
			_log.AppendLine("Branching rule: branch on the FIRST fractional variable in rank order.");
			_log.AppendLine();
			_log.AppendLine("Step 2: Explore sub-problems (depth first with backtracking)");
			_log.AppendLine("----------------------------------------------------------");
			_log.AppendLine();
		}

		private void WriteNode(BranchNode node, int fractionalIndex, double fraction)
		{
			_log.AppendLine("  Sub-problem " + node.Label);

			string fixedText = node.FixedVariables.Count == 0
				? "none (root relaxation)"
				: string.Join(", ", node.FixedVariables
					.OrderBy(p => p.Key)
					.Select(p => _variableNames[p.Key] + " = " + p.Value));

			_log.AppendLine("    Fixed variables: " + fixedText);

			if (node.Bound.HasValue)
			{
				_log.AppendLine("    " + "Var".PadRight(10) + "Ratio".PadRight(10)
								+ "Taken".PadRight(10) + "Note");

				double remaining = _capacity;
				foreach (KeyValuePair<int, int> pair in node.FixedVariables)
				{
					if (pair.Value == 1)
						remaining -= _weights[pair.Key];
				}

				foreach (int i in _rankOrder)
				{
					string taken;
					string note;

					if (node.FixedVariables.ContainsKey(i))
					{
						taken = node.FixedVariables[i].ToString(CultureInfo.InvariantCulture);
						note = "fixed by branching";
					}
					else if (i == fractionalIndex)
					{
						taken = fraction.ToString("F3", CultureInfo.InvariantCulture);
						note = "fractional - branch here";
					}
					else if (_weights[i] <= remaining + Epsilon)
					{
						taken = "1";
						note = "fits in remaining capacity";
						remaining -= _weights[i];
					}
					else
					{
						taken = "0";
						note = "no capacity left";
					}

					_log.AppendLine("    " + _variableNames[i].PadRight(10)
						+ (_values[i] / _weights[i]).ToString("F3", CultureInfo.InvariantCulture).PadRight(10)
						+ taken.PadRight(10) + note);
				}

				_log.AppendLine(string.Format(CultureInfo.InvariantCulture,
					"    LP relaxation bound (z) = {0:F3}", node.Bound.Value));
			}

			_log.AppendLine("    Result: " + node.Status);
			_log.AppendLine();
		}

		private void WriteBestCandidate()
		{
			_log.AppendLine("==========================================================");
			_log.AppendLine("   BEST CANDIDATE");
			_log.AppendLine("==========================================================");

			if (_bestSolution == null)
			{
				_log.AppendLine("  No feasible integer solution was found.");
				return;
			}

			double weight = 0.0;
			for (int i = 0; i < _variableCount; i++)
				weight += _bestSolution[i] * _weights[i];

			for (int i = 0; i < _variableCount; i++)
			{
				_log.AppendLine(string.Format(CultureInfo.InvariantCulture,
					"  {0} = {1}", _variableNames[i], _bestSolution[i]));
			}

			_log.AppendLine(string.Format(CultureInfo.InvariantCulture,
				"  Optimal objective value z = {0:F3}", _bestValue));
			_log.AppendLine(string.Format(CultureInfo.InvariantCulture,
				"  Capacity used = {0:F3} of {1:F3}", weight, _capacity));
			_log.AppendLine(string.Format(CultureInfo.InvariantCulture,
				"  Sub-problems explored = {0}", _nodes.Count));
		}
	}
}