// \file Quartic.h  From the tag library -- covered under GPL licence.


#ifndef _MVLPP_QUARTIC_H_
#define _MVLPP_QUARTIC_H_

#include <cmath>
#include <cassert>
#include <algorithm>


#ifdef WIN32
// Visual Studio 2005 does not yet define the C99 cbrt function
inline double cbrt( double x ){
    return pow(fabs(x), 1.0/3.0) * (x>=0 ? 1 : -1);
}
#endif
/*
/// A function to evaluate x^4 + Bx^3 + Cx^2 + Dx + E
inline double eval_quartic(double B, double C, double D, double E, double x)
{
return E + x*(D + x*(C + x*(B + x)));
}
 */

/// A function that performs one iteration of Newton's method on the quartic x^4 + Bx^3 + Cx^2 + Dx + E
inline double NewtonQuartic(double B, double C, double D, double E, double x)
{
    double fx = E + x*(D + x*(C + x*(B + x)));
    double dx = D + x*(2*C + x*(3*B + x*4));
    return x - fx/dx;
}

///
inline int DepressedCubicRealRoots(double P, double Q, double r[])
{
    static const double third = 1.0/3.0;
    double third_P = third * P;
    double disc = 4 * (third_P*third_P*third_P) + Q*Q;
    if( disc > 0 && disc < 1e-12){
        disc = -disc;
    }
    if (disc >= 0 ) {
        double root_disc = sqrt(disc);
        double cube = Q < 0 ? -0.5 * (Q - root_disc) : -0.5 * (Q + root_disc);
        double u = cbrt(cube);
        double x = u - third_P / u;
        r[0] = x;
        return 1;
    } else {
        double y3_re = -0.5  * Q;
        double y3_im = 0.5 * sqrt(-disc);
        // y = cube root (y3)
        double theta = atan2(y3_im, y3_re) * third;
        double radius = sqrt(-third_P);
        double y_re = radius * cos(theta);
        double x = 2*y_re;
        double root_disc = sqrt(-3*x*x - 4*P);
        if (x > 0) {
            r[0] = -0.5 * (x + root_disc);
            r[1] = -0.5 * (x - root_disc);
            r[2] = x;
        } else {
            r[0] = x;
            r[1] = -0.5 * (x + root_disc);
            r[2] = -0.5 * (x - root_disc);
        }
        return 3;
    }    
}

/// A function to find the real roots of a quartic polynomial x^4 + Bx^3 + Cx^2 + Dx + E.
/// It efficiently implements the quartic formula as given by Cardano, Harriot, et al.
/// The precision of the resulting roots depends on the nature of the coefficients. 
/// Sufficient precision can be ensured by refining the resulting roots using Newton's method.
/// @param[in] B the coefficient of the cubic term
/// @param[in] C the coefficient of the quadratic term
/// @param[in] D the coefficient of the linear term
/// @param[in] E the coefficient of the constant term
/// @param[out] r an array in which 0, 2, or 4 real roots will be stored
/// @return the number of real roots
inline int FindQuarticRealRoots( double B, double C, double D, double E, double r[] )
{
    static const double third = 1.0/3.0;
    double alpha = C - 0.375 * B * B;
    double half_B = 0.5 * B;
    double beta = half_B *(half_B*half_B - C) + D;  
    double q_B = 0.25 * B;
    double four_gamma = B * (q_B *(C - 3 * q_B * q_B) - D) + 4*E;

    if (beta*beta < 1e-18) {
        double disc = alpha*alpha - four_gamma;
        if (disc < 0)
            return 0;
        double root_disc = sqrt(disc);
        {
            double disc2 = -alpha + root_disc;
            if (disc2 < 0)
                return 0;
            double root_disc2 = sqrt(0.5 * disc2);
            r[0] = -q_B - root_disc2;
            r[1] = -q_B + root_disc2;
        }
        {
            double disc2 = -alpha - root_disc;
            if (disc2 < 0)
                return 2;
            double root_disc2 = sqrt(0.5 * disc2);
            r[2] = -q_B - root_disc2;
            r[3] = -q_B + root_disc2;
        }
        return 4;
    }

    double third_alpha = alpha * third;
    double P = -0.25*(alpha * third_alpha + four_gamma);
    double Q = third_alpha * (0.25*(four_gamma - third_alpha * third_alpha)) - beta*beta*0.125;

    double dcr[3];
    int ndcr = DepressedCubicRealRoots(P,Q, dcr);    
    double disc2 = 2*(dcr[ndcr-1] - third_alpha);
    double W = sqrt(disc2);

    double disc_base = 2*alpha + disc2; //3*alpha + 2*y;
    double disc_add = 2*beta / W;

    int count = 0;
    double sr;
    if (disc_base + disc_add <= 0) {
        sr = sqrt( -disc_base-disc_add );
        r[count++] = -q_B + 0.5 * (W - sr);
        r[count++] = -q_B + 0.5 * (W + sr);
    }
    if (disc_base - disc_add <= 0) {
        sr = sqrt( -disc_base + disc_add );
        r[count++] = -q_B - 0.5 * (W + sr);	
        r[count++] = -q_B - 0.5 * (W - sr);
    }

    // G: no idea why this keeps getting hit by the autocamera tracker...
    if ( count == 0 && disc_base + disc_add <= 1e-12) {
        sr = sqrt( fabs(-disc_base-disc_add) );
        r[count++] = -q_B + 0.5 * (W - sr);
        r[count++] = -q_B + 0.5 * (W + sr);
    }

    
    return count;
}

#endif
