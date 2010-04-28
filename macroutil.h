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

#endif
