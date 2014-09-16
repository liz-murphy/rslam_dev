// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.


//  \file PrintMatrix.h  Utility routines helpful for debuging (prints matlab style).

#pragma once

#include <stdio.h>
#include <math.h>
#include <strings.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <float.h>

namespace rslam
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// C utility routine helpful for debuging (prints matlab style).
    inline void printm_nd(
            const double *M,  ///< Input: matrix M to print.
            int rows,         ///< Input: number of rows in M.
            int cols,         ///< Input: number of columns in M.
            int minc,         ///< Input: memory increment between rows.
            double scale,     ///< Input: to move the decimal point around.
            int format,       ///< Input: long or short.
            const char* name  ///< Input: string to print infront of the matrix.
            )
    {
        int i, j;
        char *pad, *scalename;

        int len = strlen( name ) + 64;
        scalename = (char*)malloc(len);
        snprintf( scalename, len, "%s%e * [ ", name, 1.0/scale );

        pad = (char*)malloc( strlen( scalename )+1 );
        memset(pad, ' ', strlen( scalename ) );
        pad[ strlen( scalename ) ] = 0;

        for(j = 0; j < rows; j++){
            (j == 0) ? printf("%s",scalename) : printf("%s", pad);
            for(i = 0; i < cols; i++){
                if( format ){
                    printf("% #-6.16lf  ", scale*M[j*minc+i]);
                }else{
                    printf("% #-6.7lf  ", scale*M[j*minc+i]);
                }
            }
            (j != rows-1) ? printf("; ... \n"):printf("];\n");
        }

        free(scalename);
        free(pad);
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// C utility routine helpful for debuging (prints matlab style).
    inline void printm_nf(
            const float *M,  ///< Input: matrix M to print.
            int rows,         ///< Input: number of rows in M.
            int cols,         ///< Input: number of columns in M.
            int minc,         ///< Input: memory increment between rows.
            double scale,     ///< Input: to move the decimal point around.
            int format,       ///< Input: long or short.
            const char* name  ///< Input: string to print infront of the matrix.
            )
    {
        int i, j;
        char *pad, *scalename;

        scalename = (char*)malloc( strlen( name ) + 64 );
        sprintf( scalename, "%s%e * [ ", name, 1.0/scale );

        pad = (char*)malloc( strlen( scalename )+1 );
        memset(pad, ' ', strlen( scalename ) );
        pad[ strlen( scalename ) ] = 0;

        for(j = 0; j < rows; j++){
            (j == 0) ? printf("%s",scalename) : printf("%s", pad);
            for(i = 0; i < cols; i++){
                if( format ){
                    printf("% #-6.16lf  ", scale*M[j*minc+i]);
                }else{
                    printf("% #-6.7lf  ", scale*M[j*minc+i]);
                }
            }
            (j != rows-1) ? printf("; ... \n"):printf("];\n");
        }

        free(scalename);
        free(pad);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Simplified wrapper around the c-interface for Eigen matrices.
    inline void Print(
            const Eigen::MatrixXd& dMatrix,
            const std::string& sLabel,
            int nLongFormat = 0,
            double dScale = 1.0
            )
    {
        std::string sName = sLabel + " = ";
        if( !(dMatrix.Flags & Eigen::RowMajorBit) ) {
            Eigen::MatrixXd m = dMatrix.transpose();
            printm_nd( m.data(), m.rows(), dMatrix.cols(), m.stride(), dScale, nLongFormat, sName.c_str() );
        }
        else{
            printm_nd( dMatrix.data(), dMatrix.rows(), dMatrix.cols(), dMatrix.stride(), dScale, nLongFormat, sName.c_str() );
        }
    }

    inline void Print(
            const Eigen::MatrixXf& dMatrix,
            const std::string& sLabel,
            int nLongFormat = 0,
            double dScale = 1.0
            )
    {
        std::string sName = sLabel + " = ";
        if( !(dMatrix.Flags & Eigen::RowMajorBit) ) {
            Eigen::MatrixXf m = dMatrix.transpose();
            printm_nf( m.data(), m.rows(), dMatrix.cols(), m.stride(), dScale, nLongFormat, sName.c_str() );
        }
        else{
            printm_nf( dMatrix.data(), dMatrix.rows(), dMatrix.cols(), dMatrix.stride(), dScale, nLongFormat, sName.c_str() );
        }
    }
}
