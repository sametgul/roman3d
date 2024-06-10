#define S_FUNCTION_NAME  ts_adp_const
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <math.h>

#define NPARAMS 0

/*====================*
 * S-function methods *
 *====================*/

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S) {
    
    ssSetNumSFcnParams(S, NPARAMS);  /* Number of expected parameters. */

    #if defined(MATLAB_MEX_FILE)
        if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
            if (ssGetErrorStatus(S) != NULL) {
              return;   
           }
        } else {
           return; /* Parameter mismatch will be reported by Simulink */
        }
    #endif

    /* None of this s-functions's parameters are tunable during simulation */
    {
        int_T i;
        for (i=0; i < NPARAMS; i++) {
            ssSetSFcnParamNotTunable(S, i);
        }
    }

    // input port information
    if (!ssSetNumInputPorts(S, 5)) return;    
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortWidth(S, 0, 6);  // qPtr 
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortWidth(S, 1, 6);  // xdesPtr
    ssSetInputPortDirectFeedThrough(S, 2, 1);
    ssSetInputPortWidth(S, 2, 7);  // control gains  
    ssSetInputPortDirectFeedThrough(S, 3, 1);
    ssSetInputPortWidth(S, 3, 9);  // gammas
    ssSetInputPortDirectFeedThrough(S, 4, 1);
    ssSetInputPortWidth(S, 4, 9);  // Phicap
	
    // output port information	
    if (!ssSetNumOutputPorts(S, 3)) return;
    ssSetOutputPortWidth(S, 0, 3); // Tau Control output
    ssSetOutputPortWidth(S, 1, 2); // End effector pos
    ssSetOutputPortWidth(S, 2, 9); // dPhicap

    ssSetNumSampleTimes(S, 1);
    
    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
    ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE));

} /* end mdlInitializeSizes */


/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    S-function is comprised of only continuous sample time elements
 */
static void mdlInitializeSampleTimes(SimStruct *S) {
    
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    
}

/* Function: mdlOutputs =======================================================
 * Abstract:
 *      Calculate the output of the task space controller, tau.
 */
static void mdlOutputs(SimStruct *S, int_T tid) {
    
    // input channels 
    InputRealPtrsType   qPtr    = ssGetInputPortRealSignalPtrs(S,0);
    InputRealPtrsType   xdPtr   = ssGetInputPortRealSignalPtrs(S,1);
    InputRealPtrsType   gainz   = ssGetInputPortRealSignalPtrs(S,2);
    InputRealPtrsType   gammaP  = ssGetInputPortRealSignalPtrs(S,3);
    InputRealPtrsType   PhicapP = ssGetInputPortRealSignalPtrs(S,4);

    // output channels
    real_T  *Tau     = ssGetOutputPortRealSignal(S,0); 
    real_T  *xPtr    = ssGetOutputPortRealSignal(S,1);
    real_T  *dPhicap = ssGetOutputPortRealSignal(S,2);

    // input variables
    real_T  q1, q2, q3, dq1, dq2, dq3;
    real_T  xd1, xd2, dxd1, dxd2, ddxd1, ddxd2;
    real_T  Kr1, Kr2, Kr3, alpha1, alpha2, ke1, ke2;
    real_T  gamma1,gamma2,gamma3,gamma4,gamma5,gamma6,gamma7,gamma8,gamma9;
    real_T  Phicap1,Phicap2,Phicap3,Phicap4,Phicap5,Phicap6,Phicap7,Phicap8,Phicap9;
    
    // assign the variables based on the input channels    
    q1     = (*qPtr[0]);
    q2     = (*qPtr[1]);
    q3     = (*qPtr[2]);
    dq1    = (*qPtr[3]);
    dq2    = (*qPtr[4]);
    dq3    = (*qPtr[5]);

    xd1   = (*xdPtr[0]);
    xd2   = (*xdPtr[1]);
    dxd1  = (*xdPtr[2]);
    dxd2  = (*xdPtr[3]);
    ddxd1 = (*xdPtr[4]);
    ddxd2 = (*xdPtr[5]);

    Kr1  = (*gainz[0]);
    Kr2  = (*gainz[1]);
    Kr3  = (*gainz[2]);
    alpha1  = (*gainz[3]);
    alpha2  = (*gainz[4]);
    ke1  = (*gainz[5]);
    ke2  = (*gainz[6]);
    
    Phicap1    = (*PhicapP[0]);
    Phicap2    = (*PhicapP[1]);
    Phicap3    = (*PhicapP[2]);
    Phicap4    = (*PhicapP[3]);
    Phicap5    = (*PhicapP[4]);
    Phicap6    = (*PhicapP[5]);
    Phicap7    = (*PhicapP[6]);
    Phicap8    = (*PhicapP[7]);
    Phicap9    = (*PhicapP[8]);
    
    gamma1    = (*gammaP[0]);
    gamma2    = (*gammaP[1]);
    gamma3    = (*gammaP[2]);
    gamma4    = (*gammaP[3]);
    gamma5    = (*gammaP[4]);
    gamma6    = (*gammaP[5]);
    gamma7    = (*gammaP[6]);
    gamma8    = (*gammaP[7]);
    gamma9    = (*gammaP[8]);
    
    real_T l1, l2, l3;   // link lenghts
    l1 = 0.15;
    l2 = 0.1;
    l3 = 0.065;
    
	real_T x1, x2;       // end effector position in task space

	real_T dx1, dx2;     // end effector velocities

	real_T j11, j12, j13,  // manipulator jacobian of the 3 link planar robot
		   j21, j22, j23;

	real_T jdot11, jdot12, jdot13,  // time derivative of the manipulator jacobian
		   jdot21, jdot22, jdot23;

	real_T psij11,  psij12,         // pseudo inverse of the jacobian
		   psij21,  psij22, 
		   psij31,  psij32;

	real_T pinvjd11, pinvjd12,      // time derivative of the pseudo inverse
		   pinvjd21, pinvjd22,
		   pinvjd31, pinvjd32;

	real_T g1, g2, g3;              // the self motion control vector

	real_T gdot1, gdot2, gdot3;     // time derivative of g vector

	real_T  w11, w12, w13,          // the null space of the manipulator jacobian
  		      w21, w22, w23,
			  w31, w32, w33;


	real_T e1, e2 ;					// end effector positioning errors

	real_T r1, r2, r3;              // filter tracking error like terms
 
    real_T deltaE1, deltaE2, Ke1, Ke2;

    real_T rho, eps, vR1, vR2, vR3;
///////////////////////////////////////////////////////////////////////////
/////////////////////////KINEMATIC CALCULATIONS////////////////////////////
/////////////////////////////////START/////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
    
    // temp. variables
    real_T dxaux11, dxaux12, dxaux21, dxaux22;
    real_T auxjd11, auxjd12, auxjd21, auxjd22;
    real_T jjt11, jjt12, jjt21, jjt22; 
    real_T detjjt;                 
    real_T invjjt11, invjjt12, invjjt21, invjjt22;
    real_T jdotjt11, jdotjt12, jdotjt21, jdotjt22;
    real_T jjdott11, jjdott12, jjdott21, jjdott22;


    real_T aux1, aux2, aux3, aux4, aux5,
         aux6, aux7, aux8, aux9, aux10;

    real_T  pjdjtp11,  pjdjtp12,  pjdjtp21,  pjdjtp22;
    real_T  pjjdtp11,  pjjdtp12,  pjjdtp21,  pjjdtp22;

    real_T  gaux1, gaux2, gaux3,
          gaux4, gaux5, gaux6 ;

    real_T  gdaux1, gdaux2, gdaux3, gdaux4, gdaux5,
          gdaux6, gdaux7, gdaux8,  gdaux9;	     


    real_T A11, A12, A21, A22, A31, A32,
         B11, B12, B21, B22, B31, B32,
         D11, D12, D21, D22, D31, D32; 


    // first the forward kinematics
    x1 = l1*cos(q1) + l2*cos(q1+q2) + l3*cos(q1+q2+q3) ;
    x2 = l1*sin(q1) + l2*sin(q1+q2) + l3*sin(q1+q2+q3) ;

    //	time derivative calculations for the end effector position
    dxaux11 =-l1*sin(q1)*dq1-l2*(dq1+dq2)*sin(q1+q2);
    dxaux12 =-l3*(dq1+dq2+dq3)*sin(q1+q2+q3);
    dxaux21 =l1*cos(q1)*dq1+l2*(dq1+dq2)*cos(q1+q2);
    dxaux22 =l3*(dq1+dq2+dq3)*cos(q1+q2+q3);

    dx1 = dxaux11 + dxaux12;
    dx2 = dxaux21 + dxaux22;

    // jacobian matrix calculations
    j11 =-l1*sin(q1)-l2*sin(q1+q2)-l3*sin(q1+q2+q3);
    j12 =-l2*sin(q1+q2)-l3*sin(q1+q2+q3);
    j13 =-l3*sin(q1+q2+q3);

    j21 = l1*cos(q1)+l2*cos(q1+q2)+l3*cos(q1+q2+q3);
    j22 = l2*cos(q1+q2)+l3*cos(q1+q2+q3);
    j23 = l3*cos(q1+q2+q3);

    // time derivative of jacobian .. take the derivative of the jacobian matrix
    auxjd11 =-l1*dq1*cos(q1)-l2*(dq1+dq2)*cos(q1+q2);
    auxjd12 =-l3*(dq1+dq2+dq3)*cos(q1+q2+q3);
    auxjd21 =-l1*dq1*cos(q1)-l2*(dq1+dq2)*sin(q1+q2);
    auxjd22 =-l3*(dq1+dq2+dq3)*sin(q1+q2+q3);

    jdot11 = auxjd11 + auxjd12;
    jdot12 = -l2*(dq1+dq2)*cos(q1+q2)-l3*(dq1+dq2+dq3)*cos(q1+q2+q3);
    jdot13 = -l3*(dq1+dq2+dq3)*cos(q1+q2+q3);

    jdot21 = auxjd21 + auxjd22;
    jdot22 = -l2*(dq1+dq2)*sin(q1+q2)-l3*(dq1+dq2+dq3)*sin(q1+q2+q3);
    jdot23 = -l3*(dq1+dq2+dq3)*sin(q1+q2+q3);


    //---- jacobian pseudo inverse calculations  
    // J+ = J^T *(J*J^T)^(-1)
    // first calculate (J*J^T) 
    jjt11 = j11*j11 + j12*j12 + j13*j13;
    jjt12 = j11*j21 + j12*j22 + j13*j23;
    jjt21 = jjt12;
    jjt22 = j21*j21 + j22*j22 + j23*j23;

    // take the inverse of (J*J^T)
    detjjt = jjt11 * jjt22 - jjt12*jjt21 ;

    invjjt11 =  jjt22 / detjjt;
    invjjt12 = -jjt12 / detjjt;
    invjjt21 = -jjt21 / detjjt;
    invjjt22 =  jjt11 / detjjt;

    // multiply the result by J^T to calculate the pseudoinverse 
    psij11 = j11*invjjt11 + j21*invjjt21;
    psij12 = j11*invjjt12 + j21*invjjt22;
    psij21 = j12*invjjt11 + j22*invjjt21;
    psij22 = j12*invjjt12 + j22*invjjt22;
    psij31 = j13*invjjt11 + j23*invjjt21;
    psij32 = j13*invjjt12 + j23*invjjt22;


    //time derivative of pseudoinverse Jacobian
    // J+dot = jdot^T * P^-1                     ---> D Matrix
    //         - J^T * P^-1 * Jdot * J^T * P^-1  ---> A Matrix
    //         - J^T * P^-1 * J * Jdot^T * P^-1  ---> B Matrix
    //  
    // where P = (J*J^T)

    //first calculate  
    //jdotjt = (Jdot * J^T) and jjdott =  (J * Jdot^T)
    //----jdotjt
    jdotjt11 = jdot11*j11 + jdot12*j12 + jdot13*j13;
    jdotjt12 = jdot11*j21 + jdot12*j22 + jdot13*j23;
    jdotjt21 = jdot21*j11 + jdot22*j12 + jdot23*j13;
    jdotjt22 = jdot21*j21 + jdot22*j22 + jdot23*j23;


    //----jjdott
    //note that  jjdott = jdojt^T 
    jjdott11 = j11*jdot11 + j12*jdot12 + j13*jdot13;
    jjdott12 = j11*jdot21 + j12*jdot22 + j13*jdot23;
    jjdott21 = j21*jdot11 + j22*jdot12 + j23*jdot13;
    jjdott22 = j21*jdot21 + j22*jdot22 + j23*jdot23;

    // aux varibales 
    aux1 = invjjt11*invjjt11;
    aux2 = invjjt11*invjjt12;
    aux3 = invjjt11*invjjt21;
    aux4 = invjjt11*invjjt22;

    aux5 = invjjt12*invjjt12;
    aux6 = invjjt12*invjjt21;
    aux7 = invjjt12*invjjt22;

    aux8 = invjjt21*invjjt21;
    aux9 = invjjt21*invjjt22;

    aux10= invjjt22*invjjt22;

    //calculate P-1 * jdotjt * P-1  
    pjdjtp11 = aux1*jdotjt11 + aux2*jdotjt21 + aux3*jdotjt12 + aux6*jdotjt22;
    pjdjtp12 = aux2*jdotjt11 + aux5*jdotjt21 + aux4*jdotjt12 + aux7*jdotjt22; 
    pjdjtp21 = aux3*jdotjt11 + aux4*jdotjt21 + aux8*jdotjt12 + aux9*jdotjt22;
    pjdjtp22 = aux6*jdotjt11 + aux7*jdotjt21 + aux9*jdotjt12 + aux10*jdotjt22;

    //calculate P-1 * jjdott * P-1 
    pjjdtp11 = aux1*jjdott11 + aux2*jjdott21 + aux3*jjdott12 + aux6*jjdott22; 
    pjjdtp12 = aux2*jjdott11 + aux5*jjdott21 + aux4*jjdott12 + aux7*jjdott22;
    pjjdtp21 = aux3*jjdott11 + aux4*jjdott21 + aux8*jjdott12 + aux9*jjdott22;
    pjjdtp22 = aux6*jjdott11 + aux7*jjdott21 + aux9*jjdott12 + aux10*jjdott22;

    // now all intermadiate calculations for derivative 
    // of pseudo inverse is ready 
    // I will divide the calculation in 3 steps 
    // add add the 3 matrices at the end to obtain 

    // calculate Jdot^T * P^-1
    D11 = jdot11*invjjt11 + jdot21*invjjt21;
    D12 = jdot11*invjjt12 + jdot21*invjjt22;
    D21 = jdot12*invjjt11 + jdot22*invjjt21;
    D22 = jdot12*invjjt12 + jdot22*invjjt22;
    D31 = jdot13*invjjt11 + jdot23*invjjt21;
    D32 = jdot13*invjjt12 + jdot23*invjjt22;

    // calculate  = J^T * pjdjtp 
    A11 = j11*pjdjtp11 + j21*pjdjtp21;
    A12 = j11*pjdjtp12 + j21*pjdjtp22;
    A21 = j12*pjdjtp11 + j22*pjdjtp21;
    A22 = j12*pjdjtp12 + j22*pjdjtp22;
    A31 = j13*pjdjtp11 + j23*pjdjtp21;
    A32 = j13*pjdjtp12 + j23*pjdjtp22;

    // calculate  = J^T * pjjdtp 
    B11 = j11*pjjdtp11 + j21*pjjdtp21;
    B12 = j11*pjjdtp12 + j21*pjjdtp22;
    B21 = j12*pjjdtp11 + j22*pjjdtp21;
    B22 = j12*pjjdtp12 + j22*pjjdtp22;
    B31 = j13*pjjdtp11 + j23*pjjdtp21;
    B32 = j13*pjjdtp12 + j23*pjjdtp22;

    //----time derivative of pseudoinverse of jacobian
    pinvjd11 = A11  + B11 + D11;
    pinvjd12 = A12  + B12 + D12;
    pinvjd21 = A21  + B21 + D21;
    pinvjd22 = A22  + B22 + D22;
    pinvjd31 = A31  + B31 + D31;
    pinvjd32 = A32  + B32 + D32;

    // null space of the manipulator jacobian
    // ----calculation of  w = (I3 - J+*J)
    w11 = 1 - psij11*j11 + psij12*j21;
    w12 =   - psij11*j12 + psij12*j22;
    w13 =   - psij11*j13 + psij12*j23;
    w21 =   - psij21*j11 + psij22*j21;
    w22 = 1 - psij21*j12 + psij22*j22;
    w23 =   - psij21*j13 + psij22*j23;
    w31 =   - psij31*j11 + psij32*j21;
    w32 =   - psij31*j12 + psij32*j22;
    w33 = 1 - psij31*j13 + psij32*j23;

    // self motion control g = [g1 g2 g3]^T
    // I selected this to maximize manipulability 
    // if this experiment works OK try the other one,

    //g1 = g2= g3 =0;
    //gdot1 =gdot2=gdot3=0;


    g1 = 0 ;

    gaux1 = l1*l1*l2*l2 *sin(2*q2) + 2*l1*l1*l2*l3*sin(2*q2+q3);
    gaux2 = 2*l1*l1*l3*l3 *sin(2*(q2+q3)) + l1*l2*l3*l3*sin(q2+2*q3);
    gaux3 = -l1*l2*l3*l3*sin(q2);
    g2 = gaux1 + gaux2 + gaux3;

    gaux4 = -l1*l1*l2*l3*sin(q3) + l1*l1*l2*l3*sin(2*q2+q3);
    gaux5 = 2*l1*l1*l3*l3*sin(2*(q2+q3)) + 2*l2*l2*l3*l3*sin(2*q3);
    gaux6 = 2*l1*l2*l3*l3*sin(q2+2*q3);
    g3 = gaux4 + gaux5 + gaux6;

    // -----time derivative of g 
    gdot1 = 0;
    gdaux1 = l1*l1*l2*l3*cos(q2)*2*dq2;
    gdaux2 = 2*l1*l1*l2*l3*cos(2*q2+q3)*(2*dq2+dq3);
    gdaux3 = 4*l1*l1*l3*l3*cos(2*(q2+q3))*(dq2+dq3);
    gdaux4 = l1*l2*l3*l3*cos(q2+2*q3)*(dq2+2*dq3);
    gdot2 = gdaux1 + gdaux2 + gdaux3 + gdaux4;

    gdaux5 = -l1*l1*l2*l3*cos(q3)*dq3;
    gdaux6 = 2*l1*l1*l2*l3*cos(2*q2+q3)*(2*dq2+dq3);
    gdaux7 = 4*l1*l1*l3*l3*cos(2*(q2+q3))*(dq2+dq3);
    gdaux8 = 4*l2*l2*l3*l3*cos(2*q3)*dq3;
    gdaux9 = 2*l1*l2*l3*l3*cos(q2+2*q3)*(dq2+2*dq3);
    gdot3 = gdaux5 + gdaux6 + gdaux7 + gdaux8 + gdaux9;

///////////////////////////////////////////////////////////////////////////
////////////////////////CONTROLLER CALCULATIONS////////////////////////////
///////////////////////////////START///////////////////////////////////////
///////////////////////////////////////////////////////////////////////////  

    real_T  str11, str12, str13,
    str21, str22, str23;

    real_T 
    XXaux11a, XXaux12a, XXaux13a,
    XXaux11b, XXaux12b, XXaux13b, 
    XXaux1a,  XXaux1b,  XXaux11c,
    XXaux2a,  XXaux2b,  XXaux12c,
    XXaux3a,  XXaux3b,  XXaux13c,
    XXaux11d, XXaux12d, XXaux13d,

    XX1, XX2, XX3,

    YYaux11a, YYaux12a, YYaux13a,
    YYaux11b, YYaux12b, YYaux13b,

    YY1, YY2, YY3 ; 

    real_T 
    jpdotj11, jpdotj12, jpdotj13,
    jpdotj21, jpdotj22, jpdotj23,
    jpdotj31, jpdotj32, jpdotj33,

    jpjdot11, jpjdot12, jpjdot13,
    jpjdot21, jpjdot22, jpjdot23,
    jpjdot31, jpjdot32, jpjdot33;


    real_T 
    Regaux1,  RegY11,  RegY12,  RegY13,
    RegY14,   RegY15,  RegY16,  RegY17,
    RegY18,   RegY19,  RegY21,  RegY22,
    RegY23,   RegY24,  RegY25,  RegY26,
    RegY27,   RegY28,  RegY29,  RegY31,
    RegY32,   RegY33,  RegY34,  RegY35,
    RegY36,   RegY37,  RegY38,  RegY39; 

    real_T
    phiaux1, phiaux2, phiaux3, 
    phiaux4, phiaux5, phiaux6,
    phiaux7, phiaux8, phiaux9;

    real_T  YPhi1,  YPhi2,  YPhi3;
    real_T de1, de2; 

    // positioning error in task space
    e1 = xd1 - x1;
    e2 = xd2 - x2;

    de1 = dxd1 - dx1;
    de2 = dxd2 - dx2;

    //---- calculation of r vector 
    // r = J+ * (dxd + alpha*e) + (I - J+*J)*g - qdot 

    //Step 1 calcutale
    // J+ * (dxd + alpha*e)
    str11 = psij11*(dxd1 + alpha1*e1) + psij12*(dxd2 + alpha2*e2);
    str12 = psij21*(dxd1 + alpha1*e1) + psij22*(dxd2 + alpha2*e2);
    str13 = psij31*(dxd1 + alpha1*e1) + psij32*(dxd2 + alpha2*e2);

    //Step1
    // (I - J+*J)*g - qdot ;
    str21 = w11*g1 + w12*g2 + w13*g3 - dq1;
    str22 = w21*g1 + w22*g2 + w23*g3 - dq2;
    str23 = w31*g1 + w32*g2 + w33*g3 - dq3;

    r1 = str11 + str21 ;
    r2 = str12 + str22 ;
    r3 = str13 + str23 ;

    //---- regression matrix calculations 

    //step 1
    // XX = J+dot *(dxd+alpha*e) + J+*(ddxd + alpha*de) 
    //       + (-J+dot*J - J+*Jdot)*g + (I-J+J)*gdot

    XXaux11a = pinvjd11*(dxd1+alpha1*e1) + pinvjd12*(dxd2 + alpha2*e2);  
    XXaux12a = pinvjd21*(dxd1+alpha1*e1) + pinvjd22*(dxd2 + alpha2*e2);  
    XXaux13a = pinvjd31*(dxd1+alpha1*e1) + pinvjd32*(dxd2 + alpha2*e2);  

    XXaux11b = psij11*(ddxd1+alpha1*de1) + psij12*(ddxd2 + alpha2*de2);  
    XXaux12b = psij21*(ddxd1+alpha1*de1) + psij22*(ddxd2 + alpha2*de2);  
    XXaux13b = psij31*(ddxd1+alpha1*de1) + psij32*(ddxd2 + alpha2*de2);

    //calculate J+dot * J 
    jpdotj11 = pinvjd11*j11 + pinvjd12*j21;
    jpdotj12 = pinvjd11*j12 + pinvjd12*j22;
    jpdotj13 = pinvjd11*j13 + pinvjd12*j23;

    jpdotj21 = pinvjd21*j11 + pinvjd22*j21;
    jpdotj22 = pinvjd21*j12 + pinvjd22*j22;
    jpdotj23 = pinvjd21*j13 + pinvjd22*j23;

    jpdotj31 = pinvjd31*j11 + pinvjd32*j21;
    jpdotj32 = pinvjd31*j12 + pinvjd32*j22;
    jpdotj33 = pinvjd31*j13 + pinvjd32*j23;


    // calculate J+*Jdot
    jpjdot11 = psij11*jdot11 + psij12 * jdot21;
    jpjdot12 = psij11*jdot12 + psij12 * jdot22;
    jpjdot13 = psij11*jdot13 + psij12 * jdot23;

    jpjdot21 = psij21*jdot11 + psij22 * jdot21;
    jpjdot22 = psij21*jdot12 + psij22 * jdot22;
    jpjdot23 = psij21*jdot13 + psij22 * jdot23;

    jpjdot31 = psij31*jdot11 + psij32 * jdot21;
    jpjdot32 = psij31*jdot12 + psij32 * jdot22;
    jpjdot33 = psij31*jdot13 + psij32 * jdot23;


    XXaux1a = -(jpdotj11+jpjdot11)*g1 -(jpdotj12+jpjdot12)*g2 ;
    XXaux1b = -(jpdotj13+jpjdot13)*g3;
    XXaux11c = XXaux1a + XXaux1b;

    XXaux2a = -(jpdotj21+jpjdot21)*g1 -(jpdotj22+jpjdot22)*g2;
    XXaux2b = -(jpdotj23+jpjdot23)*g3;
    XXaux12c = XXaux2a + XXaux2b; 

    XXaux3a = -(jpdotj31+jpjdot31)*g1 -(jpdotj32+jpjdot32)*g2;
    XXaux3b = -(jpdotj33+jpjdot33)*g3;
    XXaux13c = XXaux3a + XXaux3b;


    XXaux11d = w11*gdot1 + w12*gdot2 + w13*gdot3;
    XXaux12d = w21*gdot1 + w22*gdot2 + w23*gdot3;
    XXaux13d = w31*gdot1 + w32*gdot2 + w33*gdot3;

    //add them up to find XX
    XX1 = XXaux11a + XXaux11b + XXaux11c + XXaux11d;
    XX2 = XXaux12a + XXaux12b + XXaux12c + XXaux12d;
    XX3 = XXaux13a + XXaux13b + XXaux13c + XXaux13d;


    //step 2
    // YY = J+*(dxd+ alpha*e) + (I-J+J)*g
    YYaux11a = psij11*(dxd1+alpha1*e1) + psij12*(dxd2 + alpha2*e2);  
    YYaux12a = psij21*(dxd1+alpha1*e1) + psij22*(dxd2 + alpha2*e2);
    YYaux13a = psij31*(dxd1+alpha1*e1) + psij32*(dxd2 + alpha2*e2);

    YYaux11b = w11*g1 + w12*g2 + w13*g3;
    YYaux12b = w21*g1 + w22*g2 + w23*g3;
    YYaux13b = w31*g1 + w32*g2 + w33*g3;

    YY1 = YYaux11a + YYaux11b;
    YY2 = YYaux12a + YYaux12b;
    YY3 = YYaux13a + YYaux13b;

    // regression matrix  ----> at the end ...
    Regaux1 = (YY1*(dq2+dq3)+YY2*(dq1+dq2+dq3)+YY3*(dq1-dq2-dq3))*sin(q1+q2);

    RegY11 = XX1;
    RegY12 = XX2;
    RegY13 = XX3;
    RegY14 = (2*XX1+XX2)*cos(q2) - (YY1*dq2 + YY2*(dq1+dq2))*sin(q2);
    RegY15 = (XX1+XX2)*cos(q2+q3) - Regaux1;
    RegY16 = (XX1)*cos(q3) - ((YY1+YY2)*dq3 + YY3*(dq1-dq2-dq3))*sin(q1+q2);
    RegY17 = dq1;
    RegY18 = 0;
    RegY19 = 0;

    RegY21 = 0;
    RegY22 = XX1+XX2;
    RegY23 = XX3;
    RegY24 = XX1*cos(q2) + YY1*dq1*sin(q2);
    RegY25 = XX1*cos(q2+q3) + (YY1*dq1 - YY2*dq3)*sin(q1+q2);
    RegY26 = (2*XX2+XX3)*cos(q3) + (YY1*dq3 - YY3*(3*dq1+dq2+dq3))*sin(q3) - YY2*dq3*sin(q1+q2);
    RegY27 = 0;
    RegY28 = dq2;
    RegY29 = 0;

    RegY31 = 0;
    RegY32 = 0;
    RegY33 = XX1+XX2+XX3;
    RegY34 = YY1*dq1*sin(q2);
    RegY35 = YY1*dq1*sin(q1+q2);
    RegY36 = XX2*cos(q3) + (-YY1*dq2 + YY2*(dq1+dq2))*sin(q3);
    RegY37 = 0;
    RegY38 = 0;
    RegY39 = dq3;


    // output channel 3
    dPhicap[0] = gamma1*(RegY11*r1 + RegY21*r2 + RegY31*r3);
    dPhicap[1] = gamma2*(RegY12*r1 + RegY22*r2 + RegY32*r3);
    dPhicap[2] = gamma3*(RegY13*r1 + RegY23*r2 + RegY33*r3);
    dPhicap[3] = gamma4*(RegY14*r1 + RegY24*r2 + RegY34*r3);
    dPhicap[4] = gamma5*(RegY15*r1 + RegY25*r2 + RegY35*r3);
    dPhicap[5] = gamma6*(RegY16*r1 + RegY26*r2 + RegY36*r3);
    dPhicap[6] = gamma7*(RegY17*r1 + RegY27*r2 + RegY37*r3);
    dPhicap[7] = gamma8*(RegY18*r1 + RegY28*r2 + RegY38*r3);
    dPhicap[8] = gamma9*(RegY19*r1 + RegY29*r2 + RegY39*r3);
    
//     Phicap1 = 0.0391522457;
//     Phicap2 = 0.0068462725;
//     Phicap3 = 0.0000223925;
//     Phicap4 = 0.0122979000;
//     Phicap5 = 0.0002583750;
//     Phicap6 = 0.0001722500;
//     Phicap7 = 0.0700;
//     Phicap8 = 0.0254;
//     Phicap9 = 0.000028785;
//     
    Phicap1 = 0.0391522457/2;
    Phicap2 = 0.0068462725/2;
    Phicap3 = 0.0000223925/2;
    Phicap4 = 0.0122979000/2;
    Phicap5 = 0.0002583750/2;
    Phicap6 = 0.0001722500/2;
    Phicap7 = 0;
    Phicap8 = 0;
    Phicap9 = 0;

    phiaux1 = RegY11*Phicap1 + RegY12*Phicap2 + RegY13*Phicap3 ;
    phiaux2 = RegY14*Phicap4 + RegY15*Phicap5 + RegY16*Phicap6;
    phiaux3 = RegY17*Phicap7 + RegY18*Phicap8 + RegY19*Phicap9;

    phiaux4 = RegY21*Phicap1 + RegY22*Phicap2 + RegY23*Phicap3;
    phiaux5 = RegY24*Phicap4 + RegY25*Phicap5 + RegY26*Phicap6;
    phiaux6 = RegY27*Phicap7 + RegY28*Phicap8 + RegY29*Phicap9;

    phiaux7 = RegY31*Phicap1 + RegY32*Phicap2 + RegY33*Phicap3;
    phiaux8 = RegY34*Phicap4 + RegY35*Phicap5 + RegY36*Phicap6;
    phiaux9 = RegY37*Phicap7 + RegY38*Phicap8 + RegY39*Phicap9;

    // ---- Yphi = RegY * Phi
    YPhi1 = phiaux1 + phiaux2 + phiaux3;
    YPhi2 = phiaux4 + phiaux5 + phiaux6;
    YPhi3 = phiaux7 + phiaux8 + phiaux9;

    deltaE1 = 2*0.005;
    deltaE2 = 2*0.005;

    Ke1 = ke1 / (deltaE1*deltaE1 - e1*e1);
    Ke2 = ke2 / (deltaE2*deltaE2 - e2*e2);
    
    //Ke1=Ke2=1;

    // output channel 1
    // ---- Control Torque input
    // Tau = YPhi + Kr*r + J^T * e
    rho = 1.5;
    eps = 0.05;
    
    vR1 = ((r1*rho*rho) / (sqrt(r1*r1*r2*r2*r3*r3)*rho+eps));
    vR2 = ((r2*rho*rho) / (sqrt(r1*r1*r2*r2*r3*r3)*rho+eps));
    vR3 = ((r3*rho*rho) / (sqrt(r1*r1*r2*r2*r3*r3)*rho+eps));
    
      
    Tau[0] = YPhi1 + Kr1*r1 + (j11*Ke1*e1 + j21*Ke2*e2) + vR1;
    Tau[1] = YPhi2 + Kr2*r2 + (j12*Ke1*e1 + j22*Ke2*e2) + vR2;
    Tau[2] = YPhi3 + Kr3*r3 + (j13*Ke1*e1 + j23*Ke2*e2) + vR3;
    
    //printf("vr1:%f\t vr2:%f\t vr3:%f\n", vR1, vR2, vR3);
    
    //output channel 2
    xPtr[0] = x1;
    xPtr[1] = x2;
    
    /*printf("Phicaps:\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",
            Phicap1,Phicap2,Phicap3,Phicap4,Phicap5,Phicap6,Phicap7,
            Phicap8,Phicap9);*/
}

/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S){}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif

/* END function Outher Loop Control */