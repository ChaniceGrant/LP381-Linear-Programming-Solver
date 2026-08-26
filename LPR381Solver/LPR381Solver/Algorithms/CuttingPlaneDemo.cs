using System;
using System.Collections.Generic;

using LPR381Solver.Models;

namespace LPR381Solver.Algorithms
{
    public static class CuttingPlaneDemo
    {
        public static void RunAll()
        {
            RunFractionalExample();
            RunAlreadyIntegerExample();
            RunRejectedBinaryExample();
        }

        public static void RunFractionalExample()
        {
            Console.WriteLine("### CASE 1: LP relaxation is fractional, cuts required");
            Console.WriteLine();

            LPModel model = BuildModel(
                new double[] { 8, 5 },
                new double[][] { new double[] { 1, 1 }, new double[] { 9, 5 } },
                new double[] { 6, 45 });

            Run(model);
        }
        public static void RunAlreadyIntegerExample()
        {
            Console.WriteLine("### CASE 2: LP relaxation is already integer, no cuts needed");
            Console.WriteLine();

            LPModel model = BuildModel(
                new double[] { 3, 2 },
                new double[][]
                {
                    new double[] { 2, 1 },
                    new double[] { 1, 1 },
                    new double[] { 1, 0 }
                },
                new double[] { 100, 80, 40 });

            Run(model);
        }

        public static void RunRejectedBinaryExample()
        {
            Console.WriteLine("### CASE 3: Binary model, should be rejected with a clear message");
            Console.WriteLine();

            LPModel model = new LPModel();
            model.ObjectiveType = "max";
            model.Variables.Add(new Variable("x1", 2, "bin"));
            model.Variables.Add(new Variable("x2", 3, "bin"));
            model.Constraints.Add(new Constraint(
                new List<double> { 11, 8 }, "<=", 40));

            Run(model);
        }

        private static LPModel BuildModel(double[] objective, double[][] rows, double[] rhs)
        {
            LPModel model = new LPModel();
            model.ObjectiveType = "max";

            for (int j = 0; j < objective.Length; j++)
                model.Variables.Add(new Variable("x" + (j + 1), objective[j], "int"));

            for (int i = 0; i < rows.Length; i++)
                model.Constraints.Add(new Constraint(new List<double>(rows[i]), "<=", rhs[i]));

            return model;
        }

        private static void Run(LPModel model)
        {
            try
            {
                CuttingPlane solver = new CuttingPlane();
                solver.Solve(model);
                Console.WriteLine(solver.Log);
            }
            catch (InvalidOperationException ex)
            {
                Console.WriteLine("This model cannot be solved with the Cutting Plane algorithm.");
                Console.WriteLine("Reason: " + ex.Message);
                Console.WriteLine();
            }
        }
    }
}
