using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace LPR381Solver.Models
{
    public class Constraint
    {
        public List<double> Coefficients { get; set; }

        public string Relation { get; set; }

        public double RightHandSide { get; set; }

        public Constraint(List<double> coefficients, string relation, double rightHandSide)
        {
            Coefficients = coefficients;
            Relation = relation;
            RightHandSide = rightHandSide;
        }
    }
}
