using System;
using System.Collections.Generic;

using LPR381Solver.Models;

namespace LPR381Solver.Algorithms
{
    public static class KnapsackDemo
    {
        public static void RunAll()
        {
            RunBriefExample();
            RunTightCapacity();
            RunInfeasible();
        }

        public static void RunBriefExample()
        {
            Console.WriteLine("### CASE 1: Knapsack IP from the project brief");
            Console.WriteLine();

            LPModel model = BuildModel(
                new double[] { 2, 3, 3, 5, 2, 4 },
                new double[] { 11, 8, 6, 14, 10, 10 },
                40);

            Run(model);
        }

        public static void RunTightCapacity()
        {
            Console.WriteLine("### CASE 2: Tight capacity, forces heavy fathoming");
            Console.WriteLine();

            LPModel model = BuildModel(
                new double[] { 2, 3, 3, 5, 2, 4 },
                new double[] { 11, 8, 6, 14, 10, 10 },
                12);

            Run(model);
        }

        public static void RunInfeasible()
        {
            Console.WriteLine("### CASE 3: No item fits, empty solution expected");
            Console.WriteLine();

            LPModel model = BuildModel(
                new double[] { 2, 3, 3 },
                new double[] { 50, 60, 70 },
                40);

            Run(model);
        }

        private static LPModel BuildModel(double[] values, double[] weights, double capacity)
        {
            LPModel model = new LPModel();
            model.ObjectiveType = "max";

            for (int i = 0; i < values.Length; i++)
            {
                model.Variables.Add(new Variable("x" + (i + 1), values[i], "bin"));
            }

            model.Constraints.Add(new Constraint(
                new List<double>(weights),
                "<=",
                capacity));

            return model;
        }

        private static void Run(LPModel model)
        {
            try
            {
                KnapsackBranchAndBound solver = new KnapsackBranchAndBound();
                solver.Solve(model);
                Console.WriteLine(solver.Log);
            }
            catch (InvalidOperationException ex)
            {
                Console.WriteLine("This model cannot be solved with Branch and Bound Knapsack.");
                Console.WriteLine("Reason: " + ex.Message);
                Console.WriteLine();
            }
        }
    }
}