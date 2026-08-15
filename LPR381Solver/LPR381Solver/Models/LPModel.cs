using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using System.Collections.Generic;

namespace LPR381Solver.Models
{
    public class LPModel
    {
        public string ObjectiveType { get; set; }

        public List<Variable> Variables { get; set; }

        public List<Constraint> Constraints { get; set; }

        public LPModel()
        {
            Variables = new List<Variable>();
            Constraints = new List<Constraint>();
        }
    }
}
