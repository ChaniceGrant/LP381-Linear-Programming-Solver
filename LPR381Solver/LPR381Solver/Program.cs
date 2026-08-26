using LPR381Solver.Models;

namespace LPR381Solver
{
    internal class Program
    {
        static void Main(string[] args)
        {
            Variable x1 = new Variable("x1", 2, "bin");

            Variable x2 = new Variable("x2", 3, "bin");

            Constraint constraint = new Constraint(
                new List<double> { 11, 8 },
                "<=",
                40
            );

            LPModel model = new LPModel();

            model.ObjectiveType = "max";

            model.Variables.Add(x1);
            model.Variables.Add(x2);

            model.Constraints.Add(constraint);

            Console.WriteLine("Model created successfully!");
            Console.WriteLine($"Objective: {model.ObjectiveType}");

            foreach (Variable variable in model.Variables)
            {
                Console.WriteLine(
                    $"{variable.Name}: {variable.ObjectiveCoefficient}, {variable.SignRestriction}"
                );
            }

            foreach (Constraint c in model.Constraints)
            {
                Console.WriteLine(
                    $"Constraint: {string.Join(" ", c.Coefficients)} {c.Relation} {c.RightHandSide}"
                );
            }
        }
    }
}
