using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using LPR381Solver.Models;
using System.Globalization; 

namespace LPR381Solver.Algorithms
{
    public class RevisedPrimalSimplex
    {
        //1. DECLARE VARIABLES
        private const double Epsilon = 1e-9;
        private const int MaxIterations = 500;
        private const double BigM = 1000000.0; 

        private List<string> _columnNames;
        private bool[] _isArtificial; //to flag which col's are artificial
        private double[,] _A;
        private double[]  _b;
        private double[] _c;
        private int _m;
        private int _n;


        //'Bookkeeping' Class:
        //Keeps track  of when an original variable becomes a multiple standard-form columns
        private class ColumnRef
        {
            public int ColumnIndex;
            public double Multiplier;
        }

        //originalVariables keep track of the original Variable objects
        private List<Variable> _originalVariables;
        private Dictionary<int, List<ColumnRef>> _variableMap;

        //we keep track if problem was a min 
        //this is bc everything internally gets changed to a max problem
        private bool _isMin;

        private int[] _basis;//stores which variable is currently basic as col nr
        private double[,] _Binv;
        private readonly StringBuilder _log = new StringBuilder();

        public string Status { get; private set; }//status of funtion solved
        public double ObjectiveValue { get; private set; }//value of z
        public Dictionary<string, double> VariableValues{get; private set;} //has variable name as key and value of it as value
        public string Log => _log.ToString();

        public void Solve(LPModel model)
        {
            Validate(model);
            BuildStandardForm(model);

            _log.AppendLine("-----------------------------------");
            _log.AppendLine(" Revides Primal Simplex Algorithm");
            _log.AppendLine("-----------------------------------");
            _log.AppendLine();
            WriteCanonicalForm(); //show canonical form before solving

            int iteration = 0;
            bool optimal = false;
            bool unbounded = false;

            while(iteration < MaxIterations)
            {
                iteration++;
                _log.AppendLine("-----------------------------------");
                _log.AppendLine("Iteration: " + iteration);
                _log.AppendLine("-----------------------------------");
                
                double[] cB = _basis.Select(colIndex => _c[colIndex]).ToArray();
                double[] y = RowVectorTimesMatrix(cB, _Binv);

                //display so user can keep up with process
                WriteProductForm(_Binv, "B^-1 (current)");
                WriteVector(y, "y = cB . B^-1 (simplex multipliers)");

                //Reduced cost for every non-basic column
                int entering = -1;
                double bestReducedCost = Epsilon;
                double[] reducedCosts = new double[_n];
                bool[] isBasicCol = new bool[_n];
                foreach (int b in _basis) isBasicCol[b] = true;

                for (int j=0; j<_n; j++)
                {
                    if(isBasicCol[j])
                    {
                        reducedCosts[j] = 0.0;
                        continue;
                    }

                    double yAj = 0.0;
                    for (int i =0; i < _m; i++)
                    {
                        yAj += y[i] * _A[i, j];
                    }

                    reducedCosts[j] = _c[j] - yAj;

                    if (reducedCosts[j] > bestReducedCost)
                    {
                        bestReducedCost = reducedCosts[j];
                        entering = j;
                    }
                }

                WriteReducedCosts(reducedCosts, isBasicCol, "Price Out row (cj - y.Aj)");

                if (entering == -1)
                {
                    optimal = true;
                    _log.AppendLine("All reduced costs <= 0. Optimal reached.");
                    _log.AppendLine();
                    break;
                }

                _log.AppendLine("Entering variable; " + _columnNames[entering] + string.Format(CultureInfo.InvariantCulture, " (reduced cost = {0:F3})", bestReducedCost));
            
                //d = B^-1 * A_entering
                double[] d = new double[_m];
                for (int i = 0; i  < _m; i++)
                {
                    double sum = 0.0;
                    for(int k = 0; k<_m; k++)
                    {
                        sum += _Binv[i, k]* _A[k,entering];
                    }
                    d[i] = sum;
                }
                double[] xB = MatrixTimesVector(_Binv, _b);

                //Ratio Test
                int leavingRow = -1;
                double bestRatio = double.PositiveInfinity;
                _log.AppendLine("Ratio test:");
                _log.AppendLine(" Row Basic Var xB d Ratio");
                
                for (int i =0; i<_m; i++)
                {
                    if (d[i]>Epsilon)
                    {
                        double ratio = xB[i]/d[i];
                        _log.AppendLine(string.Format(CultureInfo.InvariantCulture, 
                        " {0,-4} {1,-10} {2,8:F3} {3,8:F3} {4,10:F3}",
                        i, _columnNames[_basis[i]], xB[i], d[i], ratio));

                        if (ratio < bestRatio - Epsilon || (Math.Abs(ratio-bestRatio)<= Epsilon && (leavingRow == -1|| _basis[i]<_basis[leavingRow])))
                        {
                            bestRatio = ratio;
                            leavingRow = i;
                        }
                    }
                    else
                    {
                        _log.AppendLine(string.Format(CultureInfo.InvariantCulture, 
                        "{0,-4} {1,-10} {2,8:F3} {3,8:F3} - (not a candidate)",
                        i, _columnNames[_basis[i]], xB[i], d[i]));
                    }
                }

                _log.AppendLine();

                if(leavingRow == -1)
                {
                    unbounded = true;
                    _log.AppendLine("The problem is unbounded.");
                    _log.AppendLine();
                    break;
                }

                _log.AppendLine("Leaving varibale: " + _columnNames[_basis[leavingRow]]+
                "(row" +  leavingRow + ", ratio = " + bestRatio.ToString("F3", CultureInfo.InvariantCulture)
                + ")");
                _log.AppendLine();

                //Product form update: Binv_new = E * Binv
                double pivot = d[leavingRow];
                double[,] eta = new double[_m, _m];
                for(int i = 0; i < _m; i++)
                {
                    eta[i, i] =1.0;
                }
                for (int i = 0; i<_m; i++)
                {
                    eta[i, leavingRow] = (i == leavingRow) ? (1.0/pivot) : (-d[i]/pivot);
                }

                WriteProductForm(eta, "Eta matrix E (product form update for this iteration)");

                _Binv = MatrixTimesMatrix(eta, _Binv);
                _basis[leavingRow] = entering;

                _log.AppendLine();
            }

            if (!optimal && !unbounded)
            {
                Status = "Optimal Solution Found";
            }

            if (unbounded)
            {
                Status = "Unbounded";
                ObjectiveValue = double.NaN;
                VariableValues = new Dictionary<string, double>();
                WriteFinalSummary();
                return;
            }

            //Check feasibility
            //if an a varibale is lefte basic +  a positive value = infeasible
            double[] finalXB = MatrixTimesVector(_Binv, _b);
            for(int i = 0; i < _m; i++)
            {
                if(_isArtificial[_basis[i]] && finalXB[i] > 1e-6)
                {
                    Status = "Infeasible";
                    ObjectiveValue = double.NaN;
                    VariableValues = new Dictionary<string, double>();
                    _log.AppendLine("An artificial variable (" + _columnNames[_basis[i]] + ") remains basic at a positive value ("+ finalXB[i].ToString("F3",CultureInfo.InvariantCulture)+") at optimality.");
                    _log.AppendLine("The original model is infeasible");
                    _log.AppendLine();
                    WriteFinalSummary();
                    return;
                }
            }

            if (Status == null) Status = "Optimal";

            //Reconstructing variable values from standard-form solution
            double[] xStandard = new double[_n];
            for (int i = 0; i < _m; i++)
            {
                xStandard[_basis[i]] = finalXB[i];
            }

            VariableValues = new Dictionary<string, double>();
            double objective = 0.0;
            for(int i = 0; i<_originalVariables.Count; i ++)
            {
                double value = 0.0;
                foreach(ColumnRef refCol in _variableMap[i])
                value += refCol.Multiplier * xStandard[refCol.ColumnIndex];

                value = Math.Round(value, 3, MidpointRounding.AwayFromZero);
                VariableValues[_originalVariables[i].Name] = value;
                objective += _originalVariables[i].ObjectiveCoefficient*value;
            }
            ObjectiveValue = Math.Round(objective, 3, MidpointRounding.AwayFromZero);

            WriteFinalSummary();

        }

        private void Validate (LPModel model)
        {
            if (model == null) throw new ArgumentNullException(nameof(model));
            if (model.Variables == null || model.Variables.Count == 0)
                throw new InvalidOperationException("There are no decision variables");
            if (model.Constraints == null)
                throw new InvalidOperationException("There are no constraints");
            if(!string.Equals(model.ObjectiveType, "max", StringComparison.OrdinalIgnoreCase)&&
            !string.Equals(model.ObjectiveType, "min", StringComparison.OrdinalIgnoreCase))
            throw new InvalidOperationException("Objective type has to be a max or min");

            foreach (Constraint constraint in model.Constraints)
            {
                if(constraint.Coefficients.Count != model.Variables.Count)
                throw new InvalidOperationException(
                    "A constraint has a different amount of coefficients than there are amount of variables"
                );
                if(constraint.Relation != "<=" && constraint.Relation != ">=" && constraint.Relation != "=")
                throw new InvalidOperationException(
                    "Constraint relation must be <=, >= or =");
                
            }
        }

        private void BuildStandardForm(LPModel model)
        {
            _isMin = string.Equals(model.ObjectiveType, "min", StringComparison.OrdinalIgnoreCase);
            _originalVariables = model.Variables;
            _variableMap = new Dictionary<int, List<ColumnRef>>();
            _columnNames = new List<string>();
            List<double> costs = new List<double>();

            //1: creating a standard form x columns for each original variable whilst applying sign substitution
            for (int i = 0; i < model.Variables.Count; i ++)
            {
                Variable v = model.Variables[i];
                string restriction = (v.SignRestriction ?? "+").Trim().ToLowerInvariant();
                double workingCoeff = _isMin ? -v.ObjectiveCoefficient : v.ObjectiveCoefficient;
                _variableMap[i] = new List<ColumnRef>();

                switch(restriction)
                {
                    case "+":
                    case "int":
                    case "bin":
                    {
                        int col = _columnNames.Count;
                        _columnNames.Add(v.Name);
                        costs.Add(workingCoeff);
                        _variableMap[i].Add(new ColumnRef { ColumnIndex = col, Multiplier = 1.0});
                        break;
                    }
                    case "-":
                    {
                        int col = _columnNames.Count;
                        _columnNames.Add(v.Name + "'"); 
                        costs.Add(-workingCoeff);
                        _variableMap[i].Add(new ColumnRef { ColumnIndex = col, Multiplier = -1.0});
                        break;
                    }
                    case "urs":
                    {
                        int colPlus = _columnNames.Count;
                        _columnNames.Add(v.Name + "+");
                        costs.Add(workingCoeff);
                        int colMinus = _columnNames.Count;
                        _columnNames.Add(v.Name +  "-");
                        costs.Add(-workingCoeff);
                        _variableMap[i].Add(new ColumnRef { ColumnIndex = colPlus, Multiplier = 1.0});
                        _variableMap[i].Add(new ColumnRef { ColumnIndex = colMinus, Multiplier = -1.0});
                        break;
                    }
                    default:
                    throw new InvalidOperationException(
                        "Unrecognised sign restriction '"+ v.SignRestriction + "' for variable " + v.Name); 

                    }
                }

                int xColumnCount =_columnNames.Count;

            //2. building the raw x coefficient row
            List<double[]> rows = new List<double[]>();
            List<string> relations = new List<string>();
            List<double> rhsList = new List<double>();

            foreach (Constraint constraint in model.Constraints)
            {
                double[] row = new double[xColumnCount];
                for(int i = 0; i < model.Variables.Count; i ++)
                {
                    double a = constraint. Coefficients[i];
                    foreach (ColumnRef refCol in _variableMap[i])
                    row[refCol.ColumnIndex] += a*refCol.Multiplier;
                }

                double rhs = constraint.RightHandSide;
                string relation = constraint.Relation;

                if (rhs<0)
                {
                    for ( int j = 0; j < xColumnCount; j++) row[j] = -row[j];
                    rhs = -rhs;
                    if(relation == "<=") relation = ">=";
                    else if (relation == ">=") relation = "<=";
                }

                rows.Add(row);
                relations.Add(relation);
                rhsList.Add(rhs);
            }

            //2.2: add an explicit x <= 1 for every binary variable there is 
            for(int i = 0; i<model.Variables.Count; i++)
            {
                string restriction = (model.Variables[i].SignRestriction ?? "+").Trim().ToLowerInvariant();
                if(restriction != "bin") continue;

                double[] row = new double[xColumnCount];
                row[_variableMap[i][0].ColumnIndex] = 1.0;
                rows.Add(row);
                relations.Add("<=");
                rhsList.Add(1.0);
            }

            _m = rows.Count;

            //3: append the slack or surplus or artificial variables' columns
            List<List<double>> fullRows = rows.Select(r => r.ToList()).ToList();
            List<int> basisColumns = new List<int>();
            _isArtificial = new bool[0];

            for(int rIdx = 0; rIdx < _m; rIdx++)
            {
                string relation = relations[rIdx];

                if(relation =="<=")
                {
                    int col;
                    AddZeroColumn(fullRows, out col);
                    fullRows[rIdx][col] = 1.0;
                    _columnNames.Add("s" + (rIdx + 1));
                    costs.Add(0.0);
                    basisColumns.Add(col);
                }
                else if (relation == ">=")
                {
                    int surplusCol;
                    AddZeroColumn(fullRows, out surplusCol);
                    fullRows[rIdx][surplusCol] = -1.0;
                    _columnNames.Add("e" + (rIdx + 1));
                    costs.Add(0.0);

                    int artificialCol;
                    AddZeroColumn(fullRows, out artificialCol);
                    fullRows[rIdx][artificialCol] = 1.0;
                    _columnNames.Add("a" + (rIdx + 1));
                    costs.Add(-BigM);
                    basisColumns.Add(artificialCol);
                }
                else
                {
                    int artificialCol;
					AddZeroColumn(fullRows, out artificialCol);
					fullRows[rIdx][artificialCol] = 1.0;
					_columnNames.Add("a" + (rIdx + 1));
					costs.Add(-BigM);
					basisColumns.Add(artificialCol);
                }
            }

            _n = _columnNames.Count;
            _isArtificial= new bool[_n];
            for (int j = 0; j<_n; j++) _isArtificial[j] = _columnNames[j].StartsWith("a");

            _A =new double[_m, _n];
            for ( int i = 0; i<_m; i++)
            for(int j = 0; j<_n; j++)
            _A[i,j] = fullRows[i][j];

            _b = rhsList.ToArray();
            _c = costs.ToArray();

            _basis = basisColumns.ToArray();
            _Binv = new double[_m, _m];
            for (int i = 0; i < _m; i++) _Binv[i, i] = 1.0; 
            }

            //Append a new only zeros col to every row and return the index of it
            private static void AddZeroColumn(List<List<double>> rows, out int newColumnIndex)
            {
                newColumnIndex = rows[0].Count;
                foreach(List<double> row in rows) row.Add(0.0);
            }

            //Small matrix  helpers
            private static double[] RowVectorTimesMatrix(double[] row, double[,] matrix)
            {
                int m = row.Length;
                double[] result = new double[m];
                for (int j=0; j< m; j++)
                {
                    double sum =  0.0;
                    for (int i=0; i<m;  i++) sum += row[i] * matrix[i,j];
                    result[j] = sum;
                }

                return result;
            }

            private static double[] MatrixTimesVector(double[,] matrix, double[] vector)
            {
                int m = vector.Length;
                double[] result = new double[m];
                for (int i =0 ; i<m; i++)
                {
                    double sum =0.0;
                    for(int k = 0; k<m; k++) sum += matrix[i,k] * vector[k];
                    result[i] = sum;
                }
                return result;
            }
            
            private static double[,] MatrixTimesMatrix(double[,] a, double[,] b)
            {
                int m = a.GetLength(0);
                double[,] result = new double[m, m];
                for (int i = 0; i < m; i++)
                {
                    for (int j = 0; j < m; j++)
                    {
                         double sum = 0.0;
                         for (int k = 0; k < m; k++) sum += a[i, k] * b[k, j];
                        result[i, j] = sum;
                    }
                }
                return result;
            }

            //Logging our helpers
            private void WriteCanonicalForm()
            {
                _log.AppendLine("Canonical form (standard form with a slack or surplus or artificial):");
                _log.Append(" ").Append(_isMin ?  "min" : "max").Append(" z = ");
                for(int j = 0; j<_n; j++)
                {
                    if(Math.Abs(_c[j]) < Epsilon && _columnNames[j][0] != 'x') continue;
                    _log.Append(string.Format(CultureInfo.InvariantCulture,
                    " {0}{1:F3}{2}", _c[j] >= 0 ? "+" : "-", Math.Abs(_c[j]), _columnNames[j]));
                }

                _log.AppendLine();

                for (int i = 0; i<_m; i++)
                {
                    _log.Append(" ");
                    for (int j = 0; j < _n; j++)
				{
					if (Math.Abs(_A[i, j]) < Epsilon) continue;
					_log.Append(string.Format(CultureInfo.InvariantCulture,
						" {0}{1:F3}{2}", _A[i, j] >= 0 ? "+" : "-", Math.Abs(_A[i, j]), _columnNames[j]));
				}
				_log.AppendLine(string.Format(CultureInfo.InvariantCulture, " = {0:F3}", _b[i]));
			    }
                _log.AppendLine();

                _log.AppendLine("Initial basis:" + string.Join(",", _basis.Select(b=> _columnNames[b])));
                _log.AppendLine(string.Format(CultureInfo.InvariantCulture, "Big-M value used: {0:F0}", BigM));
                _log.AppendLine();
            }

            private void WriteProductForm(double[,] matrix, string title)
            {
                _log.AppendLine(title +  ":");
                int rows = matrix.GetLength(0);
                int cols = matrix.GetLength(1);
                for(int i =0; i<rows; i++)
                {
                    _log.Append(" [");
                    for(int j = 0; j<cols; j++)
                    {
                        _log.Append(matrix[i,j].ToString("F3", CultureInfo.InvariantCulture).PadLeft(9));
                        if(j<cols -1) _log.Append(" ");
                    }
                    _log.AppendLine("]");
                }
                _log.AppendLine();
            }

            private void WriteVector(double[] vector, string title)
            {
                _log.Append(title + ": [");
                for(int i = 0; i<vector.Length; i++)
                {
                    _log.Append(vector[i].ToString("F3", CultureInfo.InvariantCulture));
                    if(i<vector.Length - 1) _log.Append(", ");
                }
                _log.AppendLine("]");
                _log.AppendLine();
            }

            private void WriteReducedCosts(double[] reducedCosts, bool[] isBasicCol, string title)
            {
                _log.AppendLine(title + ":");
                for (int j = 0; j<_n; j++)
                {
                    if (isBasicCol[j]) continue;
                    _log.AppendLine(" " + _columnNames[j].PadRight(8)+ " : "+
                    reducedCosts[j].ToString("F3", CultureInfo.InvariantCulture));
                }
                _log.AppendLine();
            }

            private void WriteFinalSummary()
            {
                _log.AppendLine("----------------------------");
                _log.AppendLine(" RESULT : "+Status);
                _log.AppendLine("----------------------------");

                if (Status == "Optimal")
                {
                    foreach(KeyValuePair<string, double> kv in VariableValues)
                    _log.AppendLine(string.Format(CultureInfo.InvariantCulture, " {0} = {1:F3}", kv.Key, kv.Value));
                    _log.AppendLine(string.Format(CultureInfo.InvariantCulture, " Objective value z = {0:F3}", ObjectiveValue));
                }
            }
            }
            
        }

