/** Ico controller for the pololu 3pi
 *
 *           \class  Uico
 *
 *                   A bandpass neuron can emulate a filter
 *                   according to:\n
 *                   \f[
 *                   h(t)=\frac{1}{b} e^{at} \sin{(bt)}
 *                   \f]
 *                   with \f$a=\frac{\pi f}{q}\f$
 *                   and \f$b=\sqrt{(2 \pi f)^2-a^2}\f$ yields
 *                   into:
 *                   \f[
 *                   y[(n+1)T] =
 *                   -2 \cdot e^{-a T} cos{b T} y[nY] -
 *                   e^{-2 \cdot a T} y[(n-1)T ]+
 *                   \frac{1}{b}e^{aT}\sin{(bT)} x[nT]
 *                   \f]
 *                   The last factor \f$x[n T]\f$ is set to 1
 *                   (irrelevant because of the normalization) and T
 *                   is set to 1, too.\n
 *
 *                   \b f (default) = 0.1 \n
 *                   \b q (default) = 0.51 \n
 *                   \b normalize (default) = true \n
 *
 *                   The higher the frequency the sharper the peak\n
 *                   The higher the quality the more is the filter
 *                   oscillating
 *
 *            \date  01/03/2007 09:26:11 PM CET
 *
 *         \version  1.0
 *          \author  Paolo Di Prodi (kolo), epokh@elec.gla.ac.uk
 *                   University of Glasgow
 *
 */

#ifndef Uico_h_
#define Uico_h_

// =====================================================================================
// System Includes
// =====================================================================================

//
// these are necessary for data types associated with events and tasks, and the
// i2c function prototypes
//

//
// this has definitions for the C++ classes
//


#include "stdafx.h"


#define LEFT_SYN  0
#define RIGHT_SYN 1

#define DISTAL_L  0
#define DISTAL_R  1

#define PROXIMAL_L  2
#define PROXIMAL_R  3

//approximation of PI GREEK
#define PI 3.14159265

/*! The bias of the robot or inertia that drives him*/

#define delay_size 10

// =====================================================================================
// Forward class declarations
// =====================================================================================


// =====================================================================================
// =====================================================================================
class Uico
{

  private:
    static float  DEF_F;
    static float  DEF_Q;
	static const int max_pwr_motor=100;

    /*! Synaptic weights x 4 connections 2 distal + 2 proximal*/
    float synaptic_weights[4];
    int d_thresh,r_max;
    unsigned short err;
    /*! the energy of the agent in this case the amount of food*/
	unsigned short energy;

  public:
    /*! x0, x1 signals: not filtered */
    int distal;
    int proximal;
	float bias;
	/*! left and right switch sensors for navigation */
	unsigned short left_bump;
	unsigned short right_bump;
	unsigned short delay_left_bump[delay_size];
	unsigned short delay_right_bump[delay_size];

    /*! u0,u1 signals: filtered */
    float u0;
	float u1;

	float ul;
	float ur;

	/*! Debug fields */
	float sumpos;
	float sumneg;

  public:

    // ====================  LIFECYCLE   =========================================

    /*! Constructor f,q */
    Uico(float f=DEF_F, float q=DEF_Q);


    // ====================  OPERATORS   =========================================

    // ====================  OPERATIONS  =========================================

	void init(int left,int right);

	void readSensors(int left,int right);

    /** Calculation of the next value
     *
     *             Calculation of the next value according to the filter
     *             equations.
     *
     *      @param  in float - The new value.
     *
     */
    void filterBP();
    void avoid(float left,float right);
    signed char getSigmValue(float value);
    
    float getDistalLeft(int k=1){return k*synaptic_weights[DISTAL_L];}
	float getDistalRight(int k=1){return k*synaptic_weights[DISTAL_R];}
	
	void setProximal(float proximal){this->proximal=proximal;}
	void setDistal(float distal){this->distal=distal;}
	
	signed char getLeftOutput(){return  nextoutput_[LEFT_SYN];}
	signed char getRightOutput(){return nextoutput_[RIGHT_SYN];}

		float getU0(){return   u0;}
		float getU1(){return   u1;}
    /** Calculation of the next value
     *
     *             Calculation of the next value according to the filter
     *             equations.
     *
     *      @param  in float - The new value.
     *
     */
    void calculate();

    /** Reseting of the neuron.
     *
     *             Reseting of the neuron.
     *
     */
    void reset();

    /** Initialization
     *
     *              Initialization of the \b denomiator_ needed for the
     *              calculation of the filter.
     *
     *      @param  f float - The frequency
     *      @param  q float - The quality
     *
     *    @remarks  The quality should be bigger than 0.51
     */
    void setFQ(float f, float q);

    /** Calculation of the normalizing factor.
     *
     *              Calculation of the normalizing factor.
     *
     *    @remarks  The 'search' for the maximum is limited to 200. This
     *              can cause trouble if the frequency is to low.
     */
    void calcNorm(int i);

    // ====================  ACCESS      =========================================


    void setBurst(bool burst) {burst_ = burst; calcNorm(LEFT_SYN);calcNorm(RIGHT_SYN);};

    void setNormalize(bool fnormalize) {normalize_ = fnormalize;};
    bool getNormalize() {return normalize_;};
	void setDistanceLimit(int d){d_thresh=d;};

    // ====================  INQUIRY     =========================================

  private:

	void delay(int D,unsigned short int* w) ;
	unsigned short int sum_delay(int D,unsigned short int* w); 

    /*! The pre-factor */
    double  denominator_x0_[2];
    double  denominator_x1_[2];
        
    /*! The output with history */
    float  buffer_x0_[2];
    float  buffer_x1_[2];
    
	/* An IIR filter for the avoidance response */
	float  delay_coeff_[2];
	float  buffer_left_[2];
	float  buffer_right_[2];
	float  buffer_out_left_[2];
	float  buffer_out_right_[2];
    

    signed char nextoutput_[2];

    /*! The normalizing factor */
    float  norm_;

    /*! True if normalizing bursts */
    bool    burst_;

    /*! A switch for normalizing */
    bool    normalize_;

  protected:
    /*! The learning rate */
    float learningRate_;

    /*! The reflex input */
    float reflex_;

    /*! A switch for (no) learning */
    bool   noLearning_;

};

#endif
