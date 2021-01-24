#include "Eigen/Eigen"
#include "osqp++.h"

using namespace Eigen;
using namespace osqp;

int main()
{
    constexpr double kInfinity = std::numeric_limits<double>::infinity();
    SparseMatrix<double> objective_matrix(2, 2);
    const Triplet<double> kTripletsP[] = {
        {0, 0, 2.0}, {1, 0, 0.5}, {0, 1, 0.5}, {1, 1, 2.0} };
    objective_matrix.setFromTriplets(std::begin(kTripletsP),
                                     std::end(kTripletsP));

    SparseMatrix<double> constraint_matrix(1, 2);
    const Triplet<double> kTripletsA[] = { {0, 0, 1.0} };
    constraint_matrix.setFromTriplets(std::begin(kTripletsA),
                                      std::end(kTripletsA));

    OsqpInstance instance;
    instance.objective_matrix = objective_matrix;
    instance.objective_vector.resize(2);
    instance.objective_vector << 1.0, 0.0;
    instance.constraint_matrix = constraint_matrix;
    instance.lower_bounds.resize(1);
    instance.lower_bounds << 1.0;
    instance.upper_bounds.resize(1);
    instance.upper_bounds << kInfinity;

    OsqpSolver solver;
    OsqpSettings settings;
    // Edit settings if appropriate.
    auto status = solver.Init(instance, settings);
    // Assuming status.ok().
    OsqpExitCode exit_code = solver.Solve();
    // Assuming exit_code == OsqpExitCode::kOptimal.
    double optimal_objective = solver.objective_value();
    Eigen::VectorXd optimal_solution = solver.primal_solution();

    return 0;
}
