/** Bandpass Neuron
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
 *                   \b normlize (default) = true \n
 *
 *                   The higher the frequency the sharper the peak\n
 *                   The higher the quality the more is the filter
 *                   oscillating
 *
 *            \date  01/03/2007 09:26:11 PM CET
 *
 *         \version  1.0
 *          \author  Paolo Di Prodi (epokh), epokh@elec.gla.ac.uk
 *                   University of Glasgow
 *
 */

// =====================================================================================
// Includes
// =====================================================================================

#include "Uico.h"


#define max(a,b) (((a) > (b)) ? (a) : (b))

// =====================================================================================
// Constructor and Destructor
// =====================================================================================

/** Constructor that sets frequency and quality.
 *
 *              Constructor that sets frequency and quality.
 *
 *      @param  f float - The frequency.
 *      @param  q float - The quality.
 *
 */
Uico::Uico(float f, float q)
{
	//debug
	sumpos=0.0;
	sumneg=0.0;

	/* the energy of the robot is 0 */
	energy=0;
	left_bump=0;
	right_bump=0;
	bias=0.0;
    /*! x0, x1 signals: not filtered */
    distal=0.0;
    proximal=0.0;
    /*! u0,u1 signals: filtered */
    u0=0.0;
	u1=0.0;

	/*! The learning rate */
    learningRate_=1.0;

    /*! The reflex input */
   reflex_=0.0;

    /*! A switch for (no) learning */
   noLearning_=false;

   /*! True if normalizing bursts */
   burst_=false;

    /*! A switch for normalizing */
    normalize_=true;

		/*! Filtered response to antennas */
	ul=0.0;
	ur=0.0;
    /*! Set the coefficients of the IIR filter associated to antennas*/
	delay_coeff_[0]=-1.05;
	delay_coeff_[1]=0.2750;

	d_thresh=100;
    setFQ(f,q);


	synaptic_weights[0]=-0.1;
	synaptic_weights[1]=0.1;
	synaptic_weights[2]=-0.1;
	synaptic_weights[3]=0.1;
	reset();
}


// =====================================================================================
// =====================================================================================

/** Initialization
 *
 *              Initialization of the \b denomiator_ needed for the
 *              calculation of the filter.
 *
 *      @param  f float - The frequency
 *      @param  q float - The quality
 *     @return   -
 *
 *    @remarks  The quality should be bigger than 0.51
 */
void Uico::setFQ(float f, float q)
{
  // If Q is ok
  if (q > 0)
  {

    double fTimesPi = f * PI * 2.0;
    double e = fTimesPi / (2.0 * q);
	printf("ftimes %f e %f \n",fTimesPi,e);
    // If root is ok
    if ((fTimesPi * fTimesPi - e * e) > 0)
    {

      float w = sqrt(fTimesPi * fTimesPi - e * e);
      denominator_x0_[0] = -2.0 * exp(-e) * cos(w);
      denominator_x0_[1] = exp(-2.0 * e);
      denominator_x1_[0] = -2.0 * exp(-e) * cos(w);
      denominator_x1_[1] = exp(-2.0 * e);
		if(normalize_==true)
      	{
		calcNorm(LEFT_SYN);
		calcNorm(RIGHT_SYN);
		}
		printf("e %f w %f \n",e,w);
    }
    // If root is bad
    else
    {
		err=1;

    }
  }
  // If Q is bad
  else
  {
		err=2;
  }
}

/** Calibrate the robot and sensors
 *
 *              Calibrate bottom QTR sensors
 *
 *     @return   -
 *
 *    @remarks  The 'search' for the maximum is limited to 200. This
 *              can cause trouble if the frequency is to low.
 */

void Uico::init(int left_port, int right_port)
{


}


/** Calculation of the normalizing factor.
 *
 *              Calculation of the normalizing factor.
 *
 *     @return   -
 *
 *    @remarks  The 'search' for the maximum is limited to 200. This
 *              can cause trouble if the frequency is to low.
 */
void Uico::calcNorm(int index)
{
  norm_ = 1.;
  // Reset of buffer
	reset();
  // Calculation of new norm
  float nnorm = 0;
  for (int i = 0; i < 1000; i++)
  {
    if(burst_)
    {  
	  	proximal=1;
      	filterBP();
	}
    else
    {
	     proximal=(i == 5) ? 1 : 0;
	     filterBP();
    }

    nnorm = max(nnorm, u0);
  }
  nextoutput_[index] = 0;

  if (nnorm != 0)
    norm_ = nnorm;

  // Reset of buffer
  reset();
}

/** Read sensors or synapses
 *
 *             Calculation of the next value according to the filter
 *             equations.
 *
 *      @param  in float - The new value.
 *     @return   -
 *
 *    @remarks   -
 */

void Uico::readSensors(int left,int right)
{


  // Left sensor is it proximal or distal?
  if(left>d_thresh && right>d_thresh)
  {
		distal=left-right;
        proximal=0;
  }
  else if(left<d_thresh && right<d_thresh)
  {
		distal=0;
        proximal=left-right;
  }
	
  if(left>r_max || right<r_max)
	  energy+=1;
}


/** Calculation of the next value
 *
 *             Calculation of the next value according to the filter
 *             equations.
 *
 *      @param  in float - The new value.
 *     @return   -
 *
 *    @remarks   -
 */
void Uico::filterBP()
{
  u0 = proximal- denominator_x0_[0] * buffer_x0_[0]- denominator_x0_[1] * buffer_x0_[1];

  buffer_x0_[1] = buffer_x0_[0];
  buffer_x0_[0] = proximal;

  u1 = distal-denominator_x1_[0] * buffer_x1_[0]- denominator_x1_[1] * buffer_x1_[1];

  buffer_x1_[1] = buffer_x1_[0];
  buffer_x1_[0] = distal;

  if (normalize_)
  {
    u0 /= norm_;
    u1 /= norm_;
  }


}

void Uico::avoid(float left,float right)
{
	
  ul = buffer_left_[1] - delay_coeff_[0]*buffer_out_left_[0]-delay_coeff_[1]*buffer_out_left_[1];
 
  buffer_out_left_[1] = buffer_out_left_[0];
  buffer_out_left_[0] = ul;
  
  buffer_left_[1] = buffer_left_[0];
  buffer_left_[0] = left;
  
  ur = buffer_right_[1] - delay_coeff_[0]*buffer_out_right_[0]-delay_coeff_[1]*buffer_out_right_[1];
 
  buffer_out_right_[1] = buffer_out_right_[0];
  buffer_out_right_[0] = ur;
  
  buffer_right_[1] = buffer_right_[0];
  buffer_right_[0] = right;

}

/** Calculation of the next output.
 *
 *              Iterates all connected synapses and calculates the \b
 *              nextoutput_ according to the ICO rule.
 *
 *     @return   -
 *
 *    @remarks  The flags are: \n
 *              \b 0 <=> Reflex input (will not be learned) \n
 *              \b 1 <=> Predictive input (will be learned)
 */
void Uico::calculate()
{
  float nextreflex =u0;

	/*! update delay lines for the contact sensors */
  	delay(delay_size,delay_left_bump);   
	delay(delay_size,delay_right_bump); 

	/*! push the last contact event */
	delay_left_bump[0]=left_bump;
	delay_right_bump[0]=right_bump;
	/*! weight the actual reflex with the number of times the sensor was active */
	unsigned int short wleft=sum_delay(delay_size,delay_left_bump)*left_bump;
	unsigned int short wright=sum_delay(delay_size,delay_left_bump)*right_bump;
	/*! compute the next output */
    float pre_LEFT= synaptic_weights[DISTAL_L]*u1+synaptic_weights[PROXIMAL_L]*u0-wleft;
    float pre_RIGHT= synaptic_weights[DISTAL_R]*u1+synaptic_weights[PROXIMAL_R]*u0-wright;

	/*! decrease the energy of the robot */
	 energy-=1;

	// And now the learning part of the weights ...
  if(noLearning_)
    return;

  // Learn and update the distal synaptic weights
  float derivReflex = nextreflex - reflex_;


  synaptic_weights[DISTAL_L]-=learningRate_ * derivReflex * u1;
  synaptic_weights[DISTAL_R]+=learningRate_ * derivReflex * u1;

  reflex_ = nextreflex;

  nextoutput_[LEFT_SYN]=getSigmValue(pre_LEFT+bias);
  nextoutput_[RIGHT_SYN]=getSigmValue(pre_RIGHT+bias);


}

signed char Uico::getSigmValue(float value){
	// the sigma is shaped using a correction factor /100 + 6
	float decay=(max_pwr_motor/100)+6;
	return (max_pwr_motor/(1+exp(-value/decay)));
}


/** Reseting of the neuron.
 *
 *             Reseting of the neuron.
 *
 *     @return   -
 *
 *    @remarks   -
 */
void Uico::reset()
{

		
	for(int k=0;k<2;k++)
	{
	 buffer_x0_[k] = 0;
	 buffer_x1_[k] = 0;
	 
	 buffer_left_[k]=0;
	 buffer_right_[k]=0;

	 buffer_out_left_[k]=0;
	 buffer_out_right_[k]=0;		
		
	}

	
  nextoutput_[LEFT_SYN] = 0;
  nextoutput_[RIGHT_SYN] = 0;

   for (int i=0; i<delay_size; i++)               /* reverse-order updating */
   {
	   delay_left_bump[i]=0;
		delay_right_bump[i]=0;
   }
}


void Uico::delay(int D,unsigned short int* w)            /* \(w[0]\) = input, \(w[D]\) = output */
{
       for (int i=D; i>=1; i--)               /* reverse-order updating */
              w[i] = w[i-1];
}

unsigned short int Uico::sum_delay(int D,unsigned short int* w)            
{
	unsigned short int sum=0;
       for (int i=0; i<D; i++)               /* reverse-order updating */
              sum+=w[i];
	   return sum/D;
}
