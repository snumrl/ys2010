
#include "mathclass.h"

namespace jhm {

#define GOLD 1.61803398874989484820
#define GLIMIT 100.0
#define TINY 1.0e-20
#define ITMAX 100
#define CGOLD 0.3819660
#define ZEPS 1.0e-10


void error( char* msg )
{
	std::cerr << "ERROR : " << msg << std::endl;
//	assert( FALSE );
}

void mnbrak(m_real& ax, m_real& bx, m_real& cx,
			m_real& fa, m_real&fb, m_real& fc, m_real (*func) (m_real))
{
	m_real ulim, u, r, q, fu, dum;

	fa = (*func) (ax);
	fb = (*func) (bx);
	if (fb > fa)
	{
		SHIFT(dum, ax, bx, dum)
		SHIFT(dum, fb, fa, dum)
	}
	cx = (bx) + GOLD * (bx - ax);
	fc = (*func) (cx);
	
	while (fb > fc)
	{
		r = (bx - ax) * (fb - fc);
		q = (bx - cx) * (fb - fa);
		u = bx - ((bx - cx) * q - (bx - ax) * r) /
			(2.0 * SIGN(MAX(fabs(q - r), TINY), q - r));
		ulim = bx + GLIMIT * (cx - bx);
		
		if ((bx - u) * (u - cx) > 0.0)
		{
			fu = (*func) (u);
			if (fu < fc)
			{
				ax = bx;
				bx = u;
				fa = fb;
				fb = fu;
				return;
			}
			else if (fu > fb)
			{
				cx = u;
				fc = fu;
				return;
			}
			
			u = cx + GOLD * (cx - bx);
			fu = (*func) (u);
		}
		else if ((cx - u) * (u - ulim) > 0.0)
		{
			fu = (*func) (u);
			if (fu < fc)
			{
				SHIFT(bx, cx, u, cx + GOLD * (cx - bx))
				SHIFT(fb, fc, fu, (*func) (u))
			}
		}
		else if ((u - ulim) * (ulim - cx) >= 0.0)
		{
			u = ulim;
			fu = (*func) (u);
		}
		else
		{
			u = cx + GOLD * (cx - bx);
			fu = (*func) (u);
		}
		
		SHIFT(ax, bx, cx, u)
		SHIFT(fa, fb, fc, fu)
	}
}

m_real brent(m_real ax, m_real bx, m_real cx, m_real (*f) (m_real), m_real tol,
	            m_real& xmin)
{
	int iter;
	m_real a, b, d, etemp, fu, fv, fw, fx, p, q, r, tol1, tol2, u, v, w, x, xm;
	m_real e = 0.0;

	a = (ax < cx ? ax : cx);
	b = (ax > cx ? ax : cx);
	x = w = v = bx;
	fw = fv = fx = (*f) (x);
	for (iter = 1; iter <= ITMAX; iter++)
	{
		xm = 0.5 * (a + b);
		tol2 = 2.0 * (tol1 = tol * fabs(x) + ZEPS);
		
		if (fabs(x - xm) <= (tol2 - 0.5 * (b - a)))
		{
			xmin = x;
			return fx;
		}
		
		if (fabs(e) > tol1)
		{
			r = (x - w) * (fx - fv);
			q = (x - v) * (fx - fw);
			p = (x - v) * q - (x - w) * r;
			q = 2.0 * (q - r);
			if (q > 0.0)
				p = -p;
			q = fabs(q);
			etemp = e;
			e = d;
			if (fabs(p) >= fabs(0.5*q*etemp) || p <= q*(a-x) || p >= q*(b-x))
				d = CGOLD * (e = (x >= xm ? a - x : b - x));
			else
			{
				d = p / q;
				u = x + d;
				if (u - a < tol2 || b - u < tol2)
					d = SIGN(tol1, xm - x);
			}
		}
		else
		{
			d = CGOLD * (e = (x >= xm ? a - x : b - x));
		}
		
		u = (fabs(d) >= tol1 ? x + d : x + SIGN(tol1, d));
		fu = (*f) (u);
		
		if (fu <= fx)
		{
			if (u >= x)	a = x;
				   else b = x;
			SHIFT(v, w, x, u)
			SHIFT(fv, fw, fx, fu)
		}
		else
		{
			if (u < x) a = u;
				  else b = u;
				  
			if (fu <= fw || w == x)
			{
				v = w;
				w = u;
				fv = fw;
				fw = fu;
			}
			else if (fu <= fv || v == x || v == w)
			{
				v = u;
				fv = fu;
			}
		}
	}
	
	error("Too many iterations in brent");
	xmin = x;
	return fx;
}

}