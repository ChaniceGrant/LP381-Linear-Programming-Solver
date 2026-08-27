using System;
using System.Collections.Generic;

using LPR381Solver.Models;

namespace LPR381Solver.Algorithms
{
    public static class IntegerAlgorithmDemos
    {
        public static void RunAll()
        {
            RunKnapsackBriefExample();
            RunKnapsackTightCapacity();
            RunKnapsackNothingFits();
            RunCuttingPlaneFractional();
            RunCuttingPlaneAlreadyInteger();
            RunCuttingPlaneRejectsBinary();
        }

        public static void RunKnapsackBriefExample()
        {
            Console.WriteLine("### KNAPSACK CASE 1: the IP from the project brief");
            Console.WriteLine();

            LpProblem problem = BuildProblem(
                true,
                new double[] { 2, 3, 3, 5, 2, 4 },
                new double[][] { new double[] { 11, 8, 6, 14, 10, 10 } },
                new double[] { 40 },
                "bin");

            RunKnapsack(problem);
        }

        public static void RunKnapsackTightCapacity()
        {
            Console.WriteLine("### KNAPSACK CASE 2: tight capacity, forces heavy fathoming");
            Console.WriteLine();

            LpProblem problem = BuildProblem(
                true,
                new double[] { 2, 3, 3, 5, 2, 4 },
                new double[][] { new double[] { 11, 8, 6, 14, 10, 10 } },
                new double[] { 12 },
                "bin");

            RunKnapsack(problem);
        }

        public static void RunKnapsackNothingFits()
        {
            Console.WriteLine("### KNAPSACK CASE 3: no item fits, empty solution expected");
            Console.WriteLine();

            LpProblem problem = BuildProblem(
                true,
                new double[] { 2, 3, 3 },
                new double[][] { new double[] { 50, 60, 70 } },
                new double[] { 40 },
                "bin");

            RunKnapsack(problem);
        }

        public static void RunCuttingPlaneFractional()
        {
            Console.WriteLine("### CUTTING PLANE CASE 1: LP relaxation is fractional, cuts required");
            Console.WriteLine();

            LpProblem problem = BuildProblem(
                true,
                new double[] { 8, 5 },
                new double[][] { new double[] { 1, 1 }, new double[] { 9, 5 } },
                new double[] { 6, 45 },
                "int");

            RunCuttingPlane(problem);
        }

        public static void RunCuttingPlaneAlreadyInteger()
        {
            Console.WriteLine("### CUTTING PLANE CASE 2: LP relaxation is already integer, no cuts needed");
            Console.WriteLine();

            LpProblem problem = BuildProblem(
                true,
                new double[] { 3, 2 },
                new double[][]
                {
                    new double[] { 2, 1 },
                    new double[] { 1, 1 },
                    new double[] { 1, 0 }
                },
                new double[] { 100, 80, 40 },
                "int");

            RunCuttingPlane(problem);
        }

        public static void RunCuttingPlaneRejectsBinary()
        {
            Console.WriteLine("### CUTTING PLANE CASE 3: binary model, should be rejected");
            Console.WriteLine();

            LpProblem problem = BuildProblem(
                true,
                new double[] { 2, 3 },
                new double[][] { new double[] { 11, 8 } },
                new double[] { 40 },
                "bin");

            RunCuttingPlane(problem);
        }

        private static LpProblem BuildProblem(
            bool isMaximisation,
            double[] objective,
            double[][] constraintRows,
            double[] rhs,
            string signRestriction)
        {
            LpProblem problem = new LpProblem();
            problem.IsMaximization = isMaximisation;

            foreach (double coefficient in objective)
            {
                problem.ObjectiveCoeffs.Add(coefficient);
                problem.SignRestrictions.Add(signRestriction);
            }

            for (int i = 0; i < constraintRows.Length; i++)
            {
                problem.ConstraintCoeffs.Add(new List<double>(constraintRows[i]));
                problem.Relations.Add("<=");
                problem.Rhs.Add(rhs[i]);
            }

            return problem;
        }

        private static void RunKnapsack(LpProblem problem)
        {
            try
            {
                KnapsackBranchAndBound solver = new KnapsackBranchAndBound();
                solver.Solve(problem);
                Console.WriteLine(solver.Log);
            }
            catch (InvalidOperationException ex)
            {
                Console.WriteLine("This model cannot be solved with Branch and Bound Knapsack.");
                Console.WriteLine("Reason: " + ex.Message);
                Console.WriteLine();
            }
        }

        private static void RunCuttingPlane(LpProblem problem)
        {
            try
            {
                CuttingPlane solver = new CuttingPlane();
                solver.Solve(problem);
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