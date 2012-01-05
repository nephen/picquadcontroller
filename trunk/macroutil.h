#ifndef MACROUTIL_H
#define MACROUTIL_H
//---------------------------------------------------------------------
// UTIL MACROS
//---------------------------------------------------------------------
#define _TRIS(pin)			pin(_TRIS_F)
#define _TRIS_F(alpha,bit)	(TRIS ## alpha ## bits.TRIS ## alpha ## bit)
#define _PORT(pin)			pin(_PORT_F)
#define _PORT_F(alpha,bit)	(PORT ## alpha ## bits.R ## alpha ## bit)
#define _LAT(pin)			pin(_LAT_F)
#define _LAT_F(alpha,bit)	(LAT ## alpha ## bits.LAT ## alpha ## bit)
#define _WPU(pin)			pin(_WPU_F)
#define _WPU_F(alpha,bit)	(WPU ## alpha ## bits.WPU ## alpha ## bit)

#define _LAT_PTR_F(alpha,bit)			&( LAT ## alpha )
#define _PORT_PTR_F(alpha,bit)			&( PORT ## alpha )
#define _BIT_F(alpha,bit)				bit

#define MIN(A,B)  (((A)<(B)) ? (A) : (B) )
#define MAX(A,B)  (((A)>(B)) ? (A) : (B) )
#define PUT_IN_RANGE(V,VMIN,VMAX) MAX(VMIN,MIN(VMAX,V))
#define MAP_TO_RANGE(V,VMIN0,VMAX0,VMIN1,VMAX1) ( (VMIN1) +  ( PUT_IN_RANGE(V,VMIN0,VMAX0) - (VMIN0) ) * ( (VMAX1) - (VMIN1) ) / ( (VMAX0) - (VMIN0) ) )

#define PI  3.141592653589793238462643383279502884197f

double squared(float x){
  return x*x;
}


double atan2deg(float y, float x){
	if(fabs(y) < 0.1 && fabs(x) < 0.1) 
		return 0.0; //avoid atan2 returning meaningless numbers
	else
		return atan2(y,x)*180.0 / PI;
}


float put_in_range(float v, float vmin, float vmax){
	return MAX(vmin,MIN(vmax,v));
}

float map_to_range(float v, float vmin0, float vmax0, float vmin1, float vmax1){
	float divider = (vmax0 - vmin0);
	if(0 == divider) divider = 1e-30;

	return vmin1 +  (put_in_range(v,vmin0,vmax0) - vmin0) * (vmax1 - vmin1) / divider; 
}

int float_to_int(float v){
	if(v>=0) 
		return (int)(v + 0.5f);
	else
		return (int)(v - 0.5f);
}

float fsign(float v){
	if(v>=0) return 1; else return -1;
}

float low_pass_filter(float vNew,float vPrev,float factor){
	return (vPrev*factor + vNew) / ( 1 + factor);
}

#endif
