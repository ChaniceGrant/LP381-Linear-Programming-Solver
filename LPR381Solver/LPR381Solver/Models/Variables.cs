using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace LPR381Solver.Models
{
    public class Variable
    {
        public string Name { get; set; }

        public double ObjectiveCoefficient { get; set; }

        public string SignRestriction { get; set; }

        public Variable(string name, double objectiveCoefficient, string signRestriction)
        {
            Name = name;
            ObjectiveCoefficient = objectiveCoefficient;
            SignRestriction = signRestriction;
        }
    }
}