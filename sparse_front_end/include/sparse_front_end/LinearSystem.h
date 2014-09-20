// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

//   Pure virtual class defines an interface for Solvers to use such as the
//   IterativeLinearizedSolver.

#ifndef _LINEAR_SYSTEM_H_
#define _LINEAR_SYSTEM_H_

class LinearSystem
{
public:
    
    virtual ~LinearSystem(){}
    
    // required functions
    virtual double Build() = 0; // Build A and b
    virtual double Solve() = 0; // Solve xnew = A\b
    virtual void   Update() = 0; // Apply newly found x
    virtual void   UndoUpdate() = 0; 

    // optional helper functions
    virtual double PreSolve() { return 0.;}
    virtual double PostSolve() { return 0.; }
};

#endif

