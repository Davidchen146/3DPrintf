#include "src/meshoperations.h"

// Adds a single variable to the solver
// Does not add any implicit constraints onto the
const MPVariable* MeshOperations::addVariable(const double &coefficient, const double &min_val, const double &max_val, const std::string &name) {
    const MPVariable* new_var = _solver->MakeIntVar(min_val, max_val, name);

    // Specify the coefficient to the objective
    MPObjective* const objective = _solver->MutableObjective();
    objective->SetCoefficient(new_var, coefficient);
    return new_var;
}

// Adds an XOR variable to the solver
const MPVariable* MeshOperations::addXORVariable(const MPVariable* var_1, const MPVariable* var_2, const double &coefficient, const std::string &name) {
    // Create the new XOR variable
    const MPVariable* xor_var = _solver->MakeIntVar(0.0, 1.0, name);

    // Add the constraints that make this an XOR variable
    double infinity = _solver->solver_infinity();
    double neg_infinity = -infinity;
    std::vector<const MPVariable*> variables {var_1, var_2, xor_var};
    std::vector<double> constraint_one_coeff {1.0, 1.0, -1.0};
    addConstraint(variables, constraint_one_coeff, 0.0, infinity);

    std::vector<double> constraint_two_coeff {1.0, -1.0, -1.0};
    addConstraint(variables, constraint_two_coeff, neg_infinity, 0.0);

    std::vector<double> constraint_three_coeff {-1.0, 1.0, -1.0};
    addConstraint(variables, constraint_three_coeff, neg_infinity, 0.0);

    std::vector<double> constraint_four_coeff {-1.0, -1.0, -1.0};
    addConstraint(variables, constraint_four_coeff, -2.0, infinity);

    // Specify the impact of this XOR variable on the objective function
    MPObjective* const objective = _solver->MutableObjective();
    objective->SetCoefficient(xor_var, coefficient);

    return xor_var;
}

// Adds a constraint to the solver
void MeshOperations::addConstraint(const std::vector<const MPVariable*> &variables, const std::vector<double> &coefficients, const double &min_val, const double &max_val, const std::string &name) {
    MPConstraint* constraint = _solver->MakeRowConstraint(min_val, max_val, name);
    // Set the coefficients
    assert(variables.size() == coefficients.size());
    for (int var = 0; var < variables.size(); var++) {
        constraint->SetCoefficient(variables[var], coefficients[var]);
    }
}

// Clears the solver
// Used in between subroutines that use the solver
void MeshOperations::clearSolver() {
    _solver->Clear();
}

// Creates the solver
void MeshOperations::preprocessSolver() {
    _solver = MPSolver::CreateSolver("SCIP");
    if (!_solver) {
        LOG(WARNING) << "SCIP solver unavailable.";
        return;
    }
}
